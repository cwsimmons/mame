// license:BSD-3-Clause
// copyright-holders:Chris Simmons

/*
 * IBM 5080 Attachment Adapter.
 *
 * Attaches an IBM 5080 Graphics System (5085 graphics processor) to an
 * IBM RT PC or PC/AT, replacing the host channel attachment (5088).
 *
 * The adapter has a 68010 processor which runs a control program loaded
 * by the host into 64KiB of shared memory, an MC68B44 DMA controller, a
 * WD1935 synchronous data link controller (SDLC, adapter is the primary
 * station), and a TMS4500A dynamic memory controller.
 *
 * Host interface (determined from the AIX and DOS device drivers):
 *   I/O 0xe90-0xe9f   write-only strobes (data ignored)
 *     0xe90  doorbell (adapter processor level 3 interrupt)
 *     0xe92  adapter reset (processor held in reset until run)
 *     0xe94  halt adapter processor
 *     0xe96  run adapter processor (restart after reset, or resume)
 *     0xe9a  clear adapter interrupt
 *     0xe9c  enable adapter interrupt
 *     0xe9e  disable adapter interrupt
 *   memory 0xe80000-0xe8ffff  shared memory (adapter processor 0x0000-0xffff)
 *   irq 10, dma channel 6 (cascade, adapter is bus master)
 *
 * Adapter processor address map:
 *   0x000000-0x00ffff  shared memory (parity checked)
 *   0x010000-0xffffff  host memory (bus master, except adapter registers)
 *   0xfce000-0xfce0ff  adapter registers
 *     0x00  W  interrupt host (either byte)
 *     0x02  W  clear doorbell interrupt (either byte)
 *     0x20  RW mc6844 (byte-wide, consecutive addresses)
 *     0x40  RW wd1935 (even addresses, register select inverted)
 *     0x80  W  line control latch (bit 6 wraps the line, others unknown)
 *     0x90  R  clear timer interrupt
 *     0x90  W  parity test (forces bad parity at 0xe090/0xe091)
 *
 * Adapter processor interrupts (autovectored):
 *   2  interval timer
 *   3  host doorbell
 *   5  wd1935
 *   6  parity error
 *
 * The serial link is exposed through a bitbanger using asynchronous HDLC
 * style framing (0x7e flags, 0x7d escape, FCS omitted), so that frames can
 * be exchanged with an external program or bridge to real hardware.
 *
 * Sources:
 *  - Reverse engineering of the 5080 attachment adapter AIX and DOS device
 *    drivers and the adapter control program (US1A5080.DTA)
 *
 * TODO:
 *  - 0xe92/0xe9c/0xe9e functions are inferred
 *  - line control latch function
 *  - dma channel 6 arbitration and bus master timing
 *  - serial link clocking and bit rate
 *  - shared interrupts
 */

#include "emu.h"
#include "5080aa.h"

#define LOG_HOST   (1U << 1)
#define LOG_LATCH  (1U << 2)
#define LOG_LINK   (1U << 3)
#define LOG_BUS    (1U << 4)

//#define VERBOSE (LOG_GENERAL|LOG_HOST|LOG_LATCH|LOG_LINK|LOG_BUS)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(ISA16_5080AA, isa16_5080aa_device, "5080aa", "IBM 5080 Attachment Adapter")

isa16_5080aa_device::isa16_5080aa_device(machine_config const &mconfig, char const *const tag, device_t *owner, u32 clock)
	: device_t(mconfig, ISA16_5080AA, tag, owner, clock)
	, device_isa16_card_interface(mconfig, *this)
	, m_cpu(*this, "cpu")
	, m_dma(*this, "dma")
	, m_sdlc(*this, "sdlc")
	, m_link(*this, "link")
	, m_ram(*this, "ram", 0x10000, ENDIANNESS_BIG)
	, m_bus_r(*this, 0xffff)
	, m_bus_w(*this)
	, m_installed(false)
{
}

void isa16_5080aa_device::device_add_mconfig(machine_config &config)
{
	// TODO: verify clock
	M68010(config, m_cpu, 8'000'000);
	m_cpu->set_addrmap(AS_PROGRAM, &isa16_5080aa_device::cpu_map);
	m_cpu->reset_cb().set(FUNC(isa16_5080aa_device::reset_w));

	MC6844(config, m_dma, 2'000'000);
	m_dma->in_memr_callback().set([this](offs_t offset) { return u8(m_ram[(offset & 0xffff) >> 1] >> (BIT(offset, 0) ? 0 : 8)); });
	m_dma->out_memw_callback().set(
		[this](offs_t offset, u8 data)
		{
			offset &= 0xffff;
			if (BIT(offset, 0))
				m_ram[offset >> 1] = (m_ram[offset >> 1] & 0xff00) | data;
			else
				m_ram[offset >> 1] = (m_ram[offset >> 1] & 0x00ff) | (u16(data) << 8);

			if ((offset & 0xff00) == 0xe000)
				m_parity[offset & 0xff] = 0;
		});
	m_dma->out_iow_callback<0>().set([this](u8 data) { m_sdlc->write(3, data); });
	m_dma->in_ior_callback<2>().set([this]() { return m_sdlc->read(4); });
	m_dma->out_txak_callback().set([this](u8 data) { m_dma_channel = data; });
	m_dma->out_dend_callback().set(FUNC(isa16_5080aa_device::dend_w));

	// TODO: verify bit rate
	WD1935(config, m_sdlc, 1'000'000);
	m_sdlc->out_intrq().set_inputline(m_cpu, M68K_IRQ_5);
	m_sdlc->out_drqo().set(m_dma, FUNC(mc6844_device::dreq_w<0>));
	m_sdlc->out_drqi().set(m_dma, FUNC(mc6844_device::dreq_w<2>));
	m_sdlc->set_frame_tx_callback(FUNC(isa16_5080aa_device::link_tx));
	m_sdlc->out_line().set(
		[this](u16 data)
		{
			// latch bit 6 wraps the line interface output back to the input
			if (BIT(m_latch, 6))
				m_sdlc->line_w(data);
		});

	BITBANGER(config, m_link, 0);
}

void isa16_5080aa_device::device_start()
{
	set_isa_device();

	m_timer = timer_alloc(FUNC(isa16_5080aa_device::timer_tick), this);
	m_poll = timer_alloc(FUNC(isa16_5080aa_device::link_poll), this);

	save_item(NAME(m_host_irq));
	save_item(NAME(m_host_ie));
	save_item(NAME(m_host_int));
	save_item(NAME(m_run));
	save_item(NAME(m_halt));
	save_item(NAME(m_latch));
	save_item(NAME(m_dend_armed));
	save_item(NAME(m_dma_channel));
	save_item(NAME(m_parity));
	save_item(NAME(m_link_escape));
	save_item(NAME(m_link_connected));

	m_host_int = false;
	m_link_escape = false;
	m_link_connected = false;
}

void isa16_5080aa_device::device_reset()
{
	if (!m_installed)
	{
		m_isa->install_device(0x0e90, 0x0e9f, *this, &isa16_5080aa_device::host_io_map);
		m_isa->install_memory(0xe8'0000, 0xe8'ffff, *this, &isa16_5080aa_device::host_mem_map);

		m_installed = true;
	}

	// the adapter processor is held until released by the host
	m_cpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
	m_cpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
	m_run = false;
	m_halt = false;

	m_host_irq = false;
	m_host_ie = false;
	host_interrupt();

	m_latch = 0;
	m_dend_armed = false;
	m_dma_channel = 0;
	std::fill(std::begin(m_parity), std::end(m_parity), 0);

	m_dma->dgrnt_w(1);

	// TODO: verify interval
	m_timer->adjust(attotime::from_usec(1500), 0, attotime::from_usec(1500));
	// the control program expects responses within about 1ms, so poll the
	// link often (this only works with emulation running near real time)
	m_poll->adjust(attotime::from_usec(25), 0, attotime::from_usec(25));

	m_link_frame.clear();
	m_link_escape = false;
}

void isa16_5080aa_device::cpu_map(address_map &map)
{
	map.unmap_value_high();

	map(0x00'0000, 0x00'ffff).ram().share(m_ram);
	map(0x00'e000, 0x00'e0ff).rw(FUNC(isa16_5080aa_device::parity_r), FUNC(isa16_5080aa_device::parity_w));

	// TODO: verify adapter register decoding, the diagnostic uses host
	// addresses 0xfe0000-0xfeffff
	map(0x01'0000, 0xff'ffff).rw(FUNC(isa16_5080aa_device::host_bus_r), FUNC(isa16_5080aa_device::host_bus_w));
	map(0xfc'0000, 0xfc'ffff).unmaprw();

	map(0xfc'e000, 0xfc'e001).lw16(NAME([this](u16 data) { status_log(); m_host_irq = true; host_interrupt(); }));
	map(0xfc'e002, 0xfc'e003).lw16(NAME([this](u16 data) { m_cpu->set_input_line(M68K_IRQ_3, CLEAR_LINE); }));
	map(0xfc'e020, 0xfc'e03f).rw(m_dma, FUNC(mc6844_device::read), FUNC(mc6844_device::write));
	map(0xfc'e040, 0xfc'e04f).lrw8(
		NAME([this](offs_t offset) { return m_sdlc->read(7 - offset); }),
		NAME([this](offs_t offset, u8 data) { m_sdlc->write(7 - offset, data); })).umask16(0xff00);
	map(0xfc'e080, 0xfc'e080).w(FUNC(isa16_5080aa_device::latch_w));
	map(0xfc'e090, 0xfc'e091).lr8(NAME([this]() { if (!machine().side_effects_disabled()) m_cpu->set_input_line(M68K_IRQ_2, CLEAR_LINE); return u8(0x00); }));
	map(0xfc'e090, 0xfc'e091).lw8(
		NAME([this](offs_t offset, u8 data)
		{
			// writes force bad parity at the corresponding shared memory
			// byte, and clear any pending parity error
			m_parity[0x90 | offset] = 1;
			m_cpu->set_input_line(M68K_IRQ_6, CLEAR_LINE);
		}));
}

void isa16_5080aa_device::host_io_map(address_map &map)
{
	map(0x0, 0xf).w(FUNC(isa16_5080aa_device::host_port_w)).flags(1);
}

void isa16_5080aa_device::host_mem_map(address_map &map)
{
	map(0x0000, 0xffff).rw(FUNC(isa16_5080aa_device::host_ram_r), FUNC(isa16_5080aa_device::host_ram_w));
}

void isa16_5080aa_device::host_port_w(offs_t offset, u16 data, u16 mem_mask)
{
	switch (offset)
	{
	case 0: // 0xe90: doorbell
		LOGMASKED(LOG_HOST, "doorbell (%s)\n", machine().describe_context());
		m_cpu->set_input_line(M68K_IRQ_3, ASSERT_LINE);
		break;

	case 1: // 0xe92: adapter reset
		LOGMASKED(LOG_HOST, "reset (%s)\n", machine().describe_context());
		m_cpu->set_input_line(INPUT_LINE_RESET, ASSERT_LINE);
		m_cpu->set_input_line(M68K_IRQ_3, CLEAR_LINE);
		m_run = false;
		reset_w(1);
		m_host_irq = false;
		host_interrupt();
		break;

	case 2: // 0xe94: halt adapter processor
		LOGMASKED(LOG_HOST, "hold (%s)\n", machine().describe_context());
		m_cpu->set_input_line(INPUT_LINE_HALT, ASSERT_LINE);
		m_halt = true;
		break;

	case 3: // 0xe96: run adapter processor (restarts after reset, otherwise resumes)
		LOGMASKED(LOG_HOST, "run (%s)\n", machine().describe_context());
		if (m_halt)
		{
			m_cpu->set_input_line(INPUT_LINE_HALT, CLEAR_LINE);
			m_halt = false;
		}
		if (!m_run)
		{
			m_cpu->set_input_line(INPUT_LINE_RESET, CLEAR_LINE);
			m_run = true;
		}
		break;

	case 5: // 0xe9a: clear adapter interrupt
		LOGMASKED(LOG_HOST, "clear interrupt (%s)\n", machine().describe_context());
		m_host_irq = false;
		host_interrupt();
		break;

	case 6: // 0xe9c: enable adapter interrupt
		LOGMASKED(LOG_HOST, "enable interrupt (%s)\n", machine().describe_context());
		m_host_ie = true;
		host_interrupt();
		break;

	case 7: // 0xe9e: disable adapter interrupt
		LOGMASKED(LOG_HOST, "disable interrupt (%s)\n", machine().describe_context());
		m_host_ie = false;
		host_interrupt();
		break;

	default:
		logerror("unknown port 0x%03x data 0x%04x (%s)\n", 0xe90 + offset * 2, data, machine().describe_context());
		break;
	}
}

void isa16_5080aa_device::status_log()
{
	if (!(VERBOSE & LOG_HOST))
		return;

	// decode the status block when it is in shared memory
	u32 const sb = (u32(m_ram[0xffc >> 1]) << 16) | m_ram[0xffe >> 1];
	if (sb < 0x1'0000 && !(sb & 1))
	{
		u16 const *const s = &m_ram[sb >> 1];
		LOGMASKED(LOG_HOST, "interrupt host status 0x%04x type 0x%02x code 0x%04x (%s)\n",
			sb, s[1] & 0xff, s[6], machine().describe_context());
	}
	else
		LOGMASKED(LOG_HOST, "interrupt host status 0x%08x (%s)\n", sb, machine().describe_context());
}

void isa16_5080aa_device::host_interrupt()
{
	bool const state = m_host_irq && m_host_ie;

	if (state != m_host_int)
	{
		m_host_int = state;
		m_isa->irq10_w(state ? ASSERT_LINE : CLEAR_LINE);
	}
}

// shared memory is byte-addressed identically by host and adapter, so the
// host (little-endian) view of each word is byte-swapped
u16 isa16_5080aa_device::host_ram_r(offs_t offset, u16 mem_mask)
{
	return swapendian_int16(m_ram[offset]);
}

void isa16_5080aa_device::host_ram_w(offs_t offset, u16 data, u16 mem_mask)
{
	data = swapendian_int16(data);
	mem_mask = swapendian_int16(mem_mask);

	COMBINE_DATA(&m_ram[offset]);

	if ((offset & 0x7f80) == 0x7000)
	{
		if (ACCESSING_BITS_8_15)
			m_parity[(offset << 1) & 0xff] = 0;
		if (ACCESSING_BITS_0_7)
			m_parity[((offset << 1) | 1) & 0xff] = 0;
	}
}

u16 isa16_5080aa_device::host_bus_r(offs_t offset, u16 mem_mask)
{
	offs_t const address = 0x01'0000 + (offset << 1);

	u16 const data = m_bus_r.isunset()
		? m_isa->memspace().read_word(address, swapendian_int16(mem_mask))
		: m_bus_r(address, swapendian_int16(mem_mask));

	return swapendian_int16(data);
}

void isa16_5080aa_device::host_bus_w(offs_t offset, u16 data, u16 mem_mask)
{
	offs_t const address = 0x01'0000 + (offset << 1);

	LOGMASKED(LOG_BUS, "bus write 0x%06x data 0x%04x mask 0x%04x (%s)\n", address, data, mem_mask, machine().describe_context());

	if (m_bus_w.isunset())
		m_isa->memspace().write_word(address, swapendian_int16(data), swapendian_int16(mem_mask));
	else
		m_bus_w(address, swapendian_int16(data), swapendian_int16(mem_mask));
}

u16 isa16_5080aa_device::parity_r(offs_t offset, u16 mem_mask)
{
	if (!machine().side_effects_disabled())
	{
		if ((ACCESSING_BITS_8_15 && m_parity[(offset << 1) | 0]) || (ACCESSING_BITS_0_7 && m_parity[(offset << 1) | 1]))
		{
			LOG("parity error 0x%04x (%s)\n", 0xe000 | (offset << 1), machine().describe_context());
			m_cpu->set_input_line(M68K_IRQ_6, ASSERT_LINE);
		}
	}

	return m_ram[0xe000 / 2 + offset];
}

void isa16_5080aa_device::parity_w(offs_t offset, u16 data, u16 mem_mask)
{
	COMBINE_DATA(&m_ram[0xe000 / 2 + offset]);

	if (ACCESSING_BITS_8_15)
		m_parity[(offset << 1) | 0] = 0;
	if (ACCESSING_BITS_0_7)
		m_parity[(offset << 1) | 1] = 0;
}

void isa16_5080aa_device::latch_w(u8 data)
{
	if (data != m_latch)
		LOGMASKED(LOG_LATCH, "latch 0x%02x (%s)\n", data, machine().describe_context());

	// bit 4 is set by the control program when starting a transmission
	// which is terminated by a chained dma block; the end of the first
	// block must not end the frame
	if (BIT(data, 4) && !BIT(m_latch, 4))
		m_dend_armed = true;
	else if (!BIT(data, 4))
		m_dend_armed = false;

	m_latch = data;
}

void isa16_5080aa_device::dend_w(int state)
{
	// only the end of a transmit (channel 0) block ends the frame
	if (state && m_dma_channel == 0)
	{
		if (m_dend_armed)
			m_dend_armed = false;
		else
			m_sdlc->eob_w(1);
	}
}

void isa16_5080aa_device::reset_w(int state)
{
	// reset instruction or adapter reset resets the peripherals
	if (state)
	{
		m_dma->reset();
		m_sdlc->reset();
		m_dma->dgrnt_w(1);

		m_latch = 0;
		m_dend_armed = false;
	}
}

TIMER_CALLBACK_MEMBER(isa16_5080aa_device::timer_tick)
{
	if (m_run)
		m_cpu->set_input_line(M68K_IRQ_2, ASSERT_LINE);
}

void isa16_5080aa_device::link_tx(std::vector<u8> const &frame)
{
	// wrapped frames do not reach the line
	if (BIT(m_latch, 6))
		return;

	LOGMASKED(LOG_LINK, "link tx %u bytes address 0x%02x control 0x%02x\n", frame.size(), frame.size() > 0 ? frame[0] : 0, frame.size() > 1 ? frame[1] : 0);

	if (!m_link->exists())
		return;

	m_link->output(0x7e);
	for (u8 const data : frame)
	{
		if (data == 0x7e || data == 0x7d)
		{
			m_link->output(0x7d);
			m_link->output(data ^ 0x20);
		}
		else
			m_link->output(data);
	}
	m_link->output(0x7e);
}

TIMER_CALLBACK_MEMBER(isa16_5080aa_device::link_poll)
{
	// treat an attached link as data set ready and carrier detect
	bool const connected = m_link->exists();
	if (connected != m_link_connected)
	{
		LOGMASKED(LOG_LINK, "link %s\n", connected ? "connected" : "disconnected");
		m_link_connected = connected;
		m_sdlc->dsr_w(connected);
		m_sdlc->cd_w(connected);
	}

	if (!connected)
		return;

	u8 buffer[256];
	u32 const length = m_link->input(buffer, sizeof(buffer));

	for (u32 i = 0; i < length; i++)
	{
		u8 const data = buffer[i];

		if (data == 0x7e)
		{
			if (!m_link_frame.empty())
			{
				LOGMASKED(LOG_LINK, "%s link rx %u bytes\n", machine().time().as_string(6), m_link_frame.size());
				m_sdlc->rx_frame(m_link_frame.data(), m_link_frame.size());
				m_link_frame.clear();
			}
			m_link_escape = false;
		}
		else if (data == 0x7d)
			m_link_escape = true;
		else
		{
			m_link_frame.push_back(m_link_escape ? (data ^ 0x20) : data);
			m_link_escape = false;
		}
	}
}
