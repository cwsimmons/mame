// license:BSD-3-Clause
// copyright-holders:Chris Simmons

/*
 * Development test harness for the IBM 5080 Attachment Adapter.
 *
 * Emulates the RT PC adapter diagnostic (psladp) host sequence without a
 * host processor: loads a 68010 control program image into the adapter
 * shared memory, starts the adapter, and reports each status posted by the
 * adapter until it reports completion or an error.
 *
 * Environment:
 *   T5080_IMAGE  path to the 48KiB control program image (loaded at 0x0)
 *   T5080_DIAG   set to write 1 to 0x1d40 (diagnostic mode, as psladp)
 */

#include "emu.h"

#include "bus/isa/isa.h"
#include "bus/isa/5080aa.h"

#include <fstream>

namespace {

class t5080_state : public driver_device
{
public:
	t5080_state(machine_config const &mconfig, device_type type, char const *tag)
		: driver_device(mconfig, type, tag)
		, m_isa(*this, "isa")
	{
	}

	void t5080(machine_config &config);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	TIMER_CALLBACK_MEMBER(start);
	TIMER_CALLBACK_MEMBER(status);
	TIMER_CALLBACK_MEMBER(timeout);
	TIMER_CALLBACK_MEMBER(jitter) {}

	void port(offs_t port) { m_isa->space(isa16_device::AS_ISA_IO).write_word(port, 0); }
	u8 mem_r(offs_t offset) { return m_isa->space(isa16_device::AS_ISA_MEM).read_byte(0xe8'0000 + offset); }
	void mem_w(offs_t offset, u8 data) { m_isa->space(isa16_device::AS_ISA_MEM).write_byte(0xe8'0000 + offset, data); }

private:
	required_device<isa16_device> m_isa;

	std::vector<u8> m_image;
	emu_timer *m_start;
	emu_timer *m_status;
	emu_timer *m_timeout;
	emu_timer *m_jitter;
	unsigned m_count;
};

static void cards(device_slot_interface &device)
{
	device.option_add("5080aa", ISA16_5080AA);
}

void t5080_state::t5080(machine_config &config)
{
	ISA16(config, m_isa, 4'772'720);
	m_isa->set_custom_spaces();
	m_isa->irq10_callback().set([this](int state) { if (state) m_status->adjust(attotime::from_usec(50)); });

	ISA16_SLOT(config, "isa1", 4'772'720, m_isa, cards, "5080aa", true);
}

void t5080_state::machine_start()
{
	m_start = timer_alloc(FUNC(t5080_state::start), this);
	m_status = timer_alloc(FUNC(t5080_state::status), this);
	m_timeout = timer_alloc(FUNC(t5080_state::timeout), this);
	m_jitter = timer_alloc(FUNC(t5080_state::jitter), this);

	char const *const path = getenv("T5080_IMAGE");
	if (path)
	{
		std::ifstream file(path, std::ios::binary);
		m_image.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
	}
	printf("t5080: image %s length 0x%x\n", path ? path : "(none)", unsigned(m_image.size()));
}

void t5080_state::machine_reset()
{
	m_count = 0;
	// start at an arbitrary point in the adapter timer phase
	char const *const phase = getenv("T5080_PHASE");
	m_start->adjust(attotime::from_msec(10) + attotime::from_usec(phase ? atoi(phase) : 0));
	m_timeout->adjust(attotime::from_seconds(120));

	// optionally fragment the scheduler timeslices (period in nanoseconds)
	if (char const *const jitter = getenv("T5080_JITTER"))
		m_jitter->adjust(attotime::from_nsec(atoi(jitter)), 0, attotime::from_nsec(atoi(jitter)));
}

TIMER_CALLBACK_MEMBER(t5080_state::start)
{
	// reset and hold
	port(0xe92);
	port(0xe94);
	port(0xe9a);

	if (!getenv("T5080_NOCLEAR"))
		for (unsigned i = 0; i < 0x10000; i++)
			mem_w(i, 0);
	for (unsigned i = 0; i < m_image.size() && i < 0x10000; i++)
		mem_w(i, m_image[i]);

	if (getenv("T5080_DIAG"))
	{
		mem_w(0x1d40, 0x00);
		mem_w(0x1d41, 0x01);
	}

	port(0xe9c);
	port(0xe96);
	printf("t5080: %s started\n", machine().time().as_string(6));
}

TIMER_CALLBACK_MEMBER(t5080_state::status)
{
	// the operational driver halts the adapter while handling status
	bool const halt = getenv("T5080_HALT");
	if (halt)
		port(0xe94);

	// status block pointer (big-endian) at 0xffc
	offs_t const sb = (u32(mem_r(0xffc)) << 24) | (u32(mem_r(0xffd)) << 16) | (u32(mem_r(0xffe)) << 8) | mem_r(0xfff);
	if (sb >= 0x1'0000)
	{
		printf("t5080: status block 0x%08x not local\n", sb);
		machine().schedule_exit();
		return;
	}

	u8 b[16];
	for (unsigned i = 0; i < 16; i++)
		b[i] = mem_r(sb + i);

	u16 const code = (b[12] << 8) | b[13];
	printf("t5080: %s #%u flag %02x %02x %02x type %02x [%02x %02x %02x %02x %02x %02x %02x %02x] code %04x\n",
		machine().time().as_string(6), ++m_count, b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9], b[10], b[11], code);

	if (b[3] == 7 || (b[3] == 5 && code == 0x00ee))
	{
		printf("t5080: %s\n", b[3] == 7 ? "FAIL" : "PASS");
		machine().schedule_exit();
		return;
	}

	// release the status block and clear the interrupt
	mem_w(sb, 0);
	port(0xe9a);
	if (halt)
		port(0xe96);
}

TIMER_CALLBACK_MEMBER(t5080_state::timeout)
{
	printf("t5080: TIMEOUT after %u statuses\n", m_count);
	machine().schedule_exit();
}

ROM_START(t5080)
ROM_END

} // anonymous namespace

COMP(2026, t5080, 0, 0, t5080, 0, t5080_state, empty_init, "<dev>", "5080 Attachment Adapter test harness", MACHINE_NO_SOUND_HW)
