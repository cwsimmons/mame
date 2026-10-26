// license:BSD-3-Clause
// copyright-holders:Christopher Simmons

#include "emu.h"

#include "cpu/m68000/m68010.h"
#include "machine/z80sio.h"
#include "machine/6840ptm.h"
#include "machine/clock.h"
#include "bus/rs232/rs232.h"

#include "ibm5085_si.h"

#include "ibm5085.lh"

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

namespace {

static constexpr offs_t RAM_BASE = 0x000000;
static constexpr offs_t RAM_SIZE = 0x080000;

// error status byte bits (PLACEHOLDER positions: take the real ones from the '576 wiring)
static constexpr u8 ERR_TIMEOUT = 0x01;   // 15 us bus timeout (U76 counter, ##364)
static constexpr u8 ERR_J1      = 0x02;   // backplane error, J1 pin 3 (upper DRAM parity)
static constexpr u8 ERR_PARITY  = 0x04;   // lower DRAM parity (U112, ##363)
static constexpr u8 ERR_SOFT    = 0x00;   // software set via FFF00A D0=0 (no status bit known)

static constexpr u8 ERR_LOCAL    = 0x70;

class ibm5085_state : public driver_device
{
public:
	ibm5085_state(const machine_config &mconfig, device_type type, const char *tag) :
		driver_device(mconfig, type, tag),
		m_att_cpu(*this, "att_cpu"),
		m_att_rom(*this, "att_rom"),
		m_bootvect(*this, "bootvect"),
		m_diag_leds(*this, "led%u", 0U),
		m_mcheck_led(*this, "mcheck_led"),
		m_periph12_uart(*this, "periph12_uart"),
		m_test_switch(*this, "SWITCHES"),
		m_si_board(*this, "si_board")
	{
    }

	void ibm50852a(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;
	virtual void device_post_load() override ATTR_COLD;

private:
	// Finders
	required_device<m68010_device> m_att_cpu;
	required_region_ptr<u16> m_att_rom;
	memory_view m_bootvect;
	output_finder<2> m_diag_leds;
	output_finder<> m_mcheck_led;
	required_device<i8274_device> m_periph12_uart;
	required_ioport m_test_switch;
	required_device<ibm5085_si_device> m_si_board;

	emu_timer *m_boot_timer = nullptr;
	TIMER_CALLBACK_MEMBER(boot_timeout);

	void att_map(address_map &map) ATTR_COLD;
	void att_iack_map(address_map &map) ATTR_COLD;

	// 0xFFF000 Block
	void parity_select_w(u8 data);
	void diag_led_w(u8 data);
	u8 testsw_r();
	void periph_baud_w(u8 data);
	void rs232_baud_w(u8 data);
	u8 err_latch_r();
	void err_latch_w(u8 data);
	u16 ext_bus_r(offs_t offset);

	u16 berr_r(offs_t offset, u16 mem_mask);
	void berr_w(offs_t offset, u16 data, u16 mem_mask);

	u8 m_parity_even = 0;
	std::unique_ptr<u8[]> m_parity_bits;
	u32 m_parity_even_count = 0;
	bool m_parity_tap_active = false;
	memory_passthrough_handler m_parity_tap;

	bool m_err_latch = false;   // U106 f13 (net344): set = error captured, further errors blocked
	u32 m_err_addr = 0;         // U100/U107/U115/U131 address snapshot
	u8 m_err_status = 0;        // status byte captured with the address
	bool m_err_type = false;    // what U106 drives onto D1 during the level 7 acknowledge

	void parity_update_tap();
	void parity_tap_w(offs_t offset, u16 &data, u16 mem_mask);
	void parity_tap_r(offs_t offset, u16 &data, u16 mem_mask);

	void error_event(u8 status_bits, offs_t address, bool gen_int);

};

void ibm5085_state::machine_start()
{
	m_boot_timer = timer_alloc(FUNC(ibm5085_state::boot_timeout), this);

	m_parity_bits = std::make_unique<u8[]>(RAM_SIZE / 8);

	save_item(NAME(m_parity_even));
	save_item(NAME(m_parity_even_count));
	save_pointer(NAME(m_parity_bits), RAM_SIZE / 8);

	save_item(NAME(m_err_latch));
	save_item(NAME(m_err_addr));
	save_item(NAME(m_err_status));
	save_item(NAME(m_err_type));

}

void ibm5085_state::machine_reset()
{
	m_bootvect.select(0);
	m_boot_timer->adjust(attotime::from_usec(4));

	m_parity_even = 0;          // reset sets PARITY_SELECTION=1 (odd)
	parity_update_tap();

	m_err_latch = false;        // reset clears U106's latch
	m_att_cpu->set_input_line(M68K_IRQ_7, CLEAR_LINE);
}

void ibm5085_state::device_post_load()
{
	m_parity_tap_active = false;
	m_parity_tap.remove();
	parity_update_tap();
}

void ibm5085_state::att_map(address_map &map)
{
	map(0x000000, 0xffffff).rw(FUNC(ibm5085_state::berr_r), FUNC(ibm5085_state::berr_w));

	map(0x000000, 0x7ffff).ram().share("ram_private");

	map(0xfcf000, 0xfcffff).m(m_si_board, FUNC(ibm5085_si_device::host_map));

	map(0xfd0000, 0xfd7fff).rom().mirror(0x008000).region("att_rom", 0);
	map(0xfe0000, 0xfe7fff).rom().mirror(0x008000).region("att_rom", 0);

	map(0xfff000, 0xfff001).w(FUNC(ibm5085_state::parity_select_w)).umask16(0x00ff).nopr();
	map(0xfff002, 0xfff003).w(FUNC(ibm5085_state::diag_led_w)).umask16(0xff00).nopr();
	map(0xfff004, 0xfff005).r(FUNC(ibm5085_state::testsw_r)).umask16(0x00ff).nopw();
	map(0xfff006, 0xfff007).w(FUNC(ibm5085_state::periph_baud_w)).umask16(0x00ff).nopr();
	map(0xfff008, 0xfff009).w(FUNC(ibm5085_state::rs232_baud_w)).umask16(0x00ff).nopr();
	map(0xfff00a, 0xfff00b).r(FUNC(ibm5085_state::err_latch_r)).w(FUNC(ibm5085_state::err_latch_w)).umask16(0x00ff);
	map(0xfff00c, 0xfff00f).r(FUNC(ibm5085_state::ext_bus_r)).nopw();

	map(0xff0040, 0xff0047).rw(m_periph12_uart, FUNC(i8274_device::ba_cd_r), FUNC(i8274_device::ba_cd_w)).umask16(0x00ff);

	map(0x000000, 0x000007).view(m_bootvect);
	m_bootvect[0](0x000000, 0x000007).rom().region("att_rom", 0);
}

void ibm5085_state::att_iack_map(address_map &map)
{
	map(0xfffff7, 0xfffff7).lr8(NAME([] () { return m68000_base_device::autovector(3); }));
	map(0xfffff9, 0xfffff9).lr8(NAME([] () { return m68000_base_device::autovector(4); }));
	map(0xfffffb, 0xfffffb).lr8(NAME([this] () -> u8 { return m_periph12_uart->inta_r(); }));
	map(0xfffffd, 0xfffffd).lr8(NAME([] () -> u8 { return 0x62; }));	// Floppy Drive
	// level 7 (error NMI): U106 drives D1 with the error type, other bits PLACEHOLDER (assumed floating high)
	map(0xffffff, 0xffffff).lr8(NAME([this] () -> u8 { m_att_cpu->set_input_line(M68K_IRQ_7, CLEAR_LINE); return 0xfd | (m_err_type ? 0x02 : 0x00); }));
}

void ibm5085_state::parity_select_w(u8 data) {
	m_parity_even = !BIT(data, 0);
	LOG("parity_reg = 0x%.2x\n", data);
	LOG("m_parity_even = %d\n", m_parity_even);
	parity_update_tap();
}

void ibm5085_state::diag_led_w(u8 data)
{
	static u8 const led_pattern[16] = {
		0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
		0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71 };   // 0x71 = "F"

	if (!BIT(data, 4)) {		// Write to upper nibble
		m_diag_leds[1] = led_pattern[data & 0x0f];
	}
	if (!BIT(data, 5)) {		// Write to lower nibble
		m_diag_leds[0] = led_pattern[data & 0x0f];
	}
	
	// Update Machine Check LED
	m_mcheck_led = BIT(data, 6);
}

u8 ibm5085_state::testsw_r()
{
	// bit 7: Test Switch (active high)
	// bit 6: Always 1 (Seemingly)
	// bit 5: W2 Pin 4 pulled high
	// bit 4: Always 1 (Same net as bit 1)
	// bit 3: Jumper W3 pins 1,2 (0 = installed, 1 = open), no hardware side effects
	// bit 2: Jumper W3 pins 3,4 (0 = installed, 1 = open), no hardware side effects
	// bit 1: Always 1 (Appears to be tied high)
	// bit 0: Mystery PAL output

	u8 data = 0x7F;
	data |= m_test_switch->read() & 0x80;

	return data;
}

void ibm5085_state::periph_baud_w(u8 data)
{

}

void ibm5085_state::rs232_baud_w(u8 data)
{

}

u8 ibm5085_state::err_latch_r()
{
	// U94's decode doesn't check R/W, so on hardware a read also clocks D0 into the latch.
	// What D0 holds during a read is unknown (floating bus); for now reads have no effect.
	return 0xff;
}

void ibm5085_state::err_latch_w(u8 data)
{
	if (BIT(data, 0))
	{
		// D0=1: clear and re-arm
		m_err_latch = false;
	}
	else
	{
		// D0=0: software set. The capture registers see the address of this access.
		error_event(ERR_SOFT | ERR_LOCAL, 0xfff00a, false);

	}
}

u16 ibm5085_state::ext_bus_r(offs_t offset)
{
	if (offset == 0) {
		return (((u16) m_err_status) << 8) | ((m_err_addr >> 16) & 0x00ff);
	} else {
		return m_err_addr & 0xffff;
	}
}

u16 ibm5085_state::berr_r(offs_t offset, u16 mem_mask)
{
	if (!machine().side_effects_disabled())
	{
		offs_t const address = (offset << 1) | (ACCESSING_BITS_8_15 ? 0 : 1);
		error_event(ERR_TIMEOUT | ERR_LOCAL, address, false);
		m_att_cpu->set_buserror_details(address, 1, m_att_cpu->get_fc());   // 1 = read
		m_att_cpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
	}
	return 0xffff;
}

void ibm5085_state::berr_w(offs_t offset, u16 data, u16 mem_mask)
{
	if (!machine().side_effects_disabled())
	{
		offs_t const address = (offset << 1) | (ACCESSING_BITS_8_15 ? 0 : 1);
		error_event(ERR_TIMEOUT | ERR_LOCAL, address, false);
		m_att_cpu->set_buserror_details(address, 0, m_att_cpu->get_fc());   // 0 = write
		m_att_cpu->set_input_line(M68K_LINE_BUSERROR, ASSERT_LINE);
	}
}

// the tap is only needed while something can mismatch: even mode selected, or even bytes still in RAM
void ibm5085_state::parity_update_tap()
{
	bool const needed = m_parity_even || m_parity_even_count;

	if (needed && !m_parity_tap_active)
	{
		m_parity_tap = m_att_cpu->space(AS_PROGRAM).install_readwrite_tap(RAM_BASE, RAM_BASE + RAM_SIZE - 1, "parity",
				[this] (offs_t offset, u16 &data, u16 mem_mask) { parity_tap_r(offset, data, mem_mask); },
				[this] (offs_t offset, u16 &data, u16 mem_mask) { parity_tap_w(offset, data, mem_mask); },
				&m_parity_tap);
		m_parity_tap_active = true;
	}
	else if (!needed && m_parity_tap_active)
	{
		m_parity_tap.remove();
		m_parity_tap_active = false;
	}
}

void ibm5085_state::parity_tap_w(offs_t offset, u16 &data, u16 mem_mask)
{
	for (unsigned b = 0; b < 2; b++)
	{
		if (!(mem_mask & (0xff00 >> (b * 8))))
			continue;                               // this byte wasn't written

		offs_t const index = (offset & ~1) + b - RAM_BASE;
		bool const was_even = BIT(m_parity_bits[index >> 3], index & 7);
		if (was_even != bool(m_parity_even))
		{
			m_parity_bits[index >> 3] ^= 1 << (index & 7);
			m_parity_even_count += m_parity_even ? 1 : -1;
		}
	}
	parity_update_tap();                            // removes the tap once everything is even again
}

void ibm5085_state::parity_tap_r(offs_t offset, u16 &data, u16 mem_mask)
{
	if (machine().side_effects_disabled())
		return;

	if (m_bootvect.entry().has_value() && offset < 0x08)
		return;

	for (unsigned b = 0; b < 2; b++)
	{
		if (!(mem_mask & (0xff00 >> (b * 8))))
			continue;

		offs_t const index = (offset & ~1) + b - RAM_BASE;
		if (BIT(m_parity_bits[index >> 3], index & 7) != m_parity_even)
		{
			LOG("parity error at %06x\n", (offset & ~1) + b);
			error_event(ERR_PARITY | ERR_LOCAL, (offset & ~1) + b, true);
			return;
		}
	}
}

void ibm5085_state::error_event(u8 status_bits, offs_t address, bool gen_int)
{
	if (m_err_latch)
		return;                 // only the first error is captured until FFF00A is written with 1

	LOG("error 0x%02x at %06x (%s)\n", status_bits, address, machine().describe_context());

	m_err_latch = true;
	m_err_addr = address;
	m_err_status = status_bits;
	m_err_type = (status_bits & ERR_PARITY) == 0;   // PLACEHOLDER: which errors set D1 is unknown

	if (gen_int)
	{
		m_att_cpu->set_input_line(M68K_IRQ_7, ASSERT_LINE);
	}
}

TIMER_CALLBACK_MEMBER(ibm5085_state::boot_timeout)
{
	m_bootvect.disable();
}

static INPUT_PORTS_START(ibm5085)
	PORT_START("SWITCHES")
	PORT_BIT( 0x80, IP_ACTIVE_HIGH, IPT_OTHER ) PORT_NAME("Test Switch") PORT_TOGGLE
	PORT_BIT( 0x7f, IP_ACTIVE_HIGH, IPT_UNUSED )
INPUT_PORTS_END

void ibm5085_state::ibm50852a(machine_config &config)
{
	M68010(config, m_att_cpu, 8'000'000);
	m_att_cpu->set_addrmap(AS_PROGRAM, &ibm5085_state::att_map);
	m_att_cpu->set_addrmap(m68000_base_device::AS_CPU_SPACE, &ibm5085_state::att_iack_map);

	IBM5085_SI(config, m_si_board);
	m_si_board->int_callback().set_inputline(m_att_cpu, M68K_IRQ_6);

	I8274(config, m_periph12_uart, 4'000'000);
	m_periph12_uart->out_int_callback().set_inputline(m_att_cpu, M68K_IRQ_5);

	clock_device &baud(CLOCK(config, "baud", 9600 * 16));
	baud.signal_handler().set(m_periph12_uart, FUNC(i8274_device::rxca_w));
	baud.signal_handler().append(m_periph12_uart, FUNC(i8274_device::txca_w));

	// U90 Port A
	rs232_port_device &port_a(RS232_PORT(config, "port_a", default_rs232_devices, nullptr));
	port_a.rxd_handler().set(m_periph12_uart, FUNC(i8274_device::rxa_w));
	port_a.cts_handler().set(m_periph12_uart, FUNC(i8274_device::ctsa_w));
	port_a.dcd_handler().set(m_periph12_uart, FUNC(i8274_device::dcda_w));
	m_periph12_uart->out_txda_callback().set(port_a, FUNC(rs232_port_device::write_txd));
	m_periph12_uart->out_rtsa_callback().set(port_a, FUNC(rs232_port_device::write_rts));
	m_periph12_uart->out_dtra_callback().set(port_a, FUNC(rs232_port_device::write_dtr));

	// U90 Port B
	rs232_port_device &port_b(RS232_PORT(config, "port_b", default_rs232_devices, nullptr));
	port_b.rxd_handler().set(m_periph12_uart, FUNC(i8274_device::rxb_w));
	port_b.cts_handler().set(m_periph12_uart, FUNC(i8274_device::ctsb_w));
	port_b.dcd_handler().set(m_periph12_uart, FUNC(i8274_device::dcdb_w));
	m_periph12_uart->out_txdb_callback().set(port_b, FUNC(rs232_port_device::write_txd));
	m_periph12_uart->out_rtsb_callback().set(port_b, FUNC(rs232_port_device::write_rts));
	m_periph12_uart->out_dtrb_callback().set(port_b, FUNC(rs232_port_device::write_dtr));

	config.set_default_layout(layout_ibm5085);

}

ROM_START(ibm50852a)
	ROM_REGION16_BE(0x8000, "att_rom", 0)
	ROM_LOAD16_BYTE("6002904p009.bin", 0x00000, 0x4000, CRC(9fadd77f) SHA1(d03036abc57d09748eac3a73e8cc9b8da18d1d53))
	ROM_LOAD16_BYTE("6002904p010.bin", 0x00001, 0x4000, CRC(99e7fd02) SHA1(adc35b570b4089b4992e06916922e9847371585d))
ROM_END

} // anonymous namespace

/*   YEAR   NAME       PARENT  COMPAT  MACHINE    INPUT   CLASS          INIT         COMPANY                             FULLNAME               FLAGS */
COMP(1986,  ibm50852a, 0,      0,      ibm50852a, ibm5085,ibm5085_state, empty_init,  "International Business Machines",  "IBM 5085 2A",         MACHINE_NOT_WORKING | MACHINE_NO_SOUND)
