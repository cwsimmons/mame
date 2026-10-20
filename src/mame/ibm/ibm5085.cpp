// license:BSD-3-Clause
// copyright-holders:Christopher Simmons

#include "emu.h"

#include "cpu/m68000/m68010.h"
#include "machine/z80sio.h"
#include "machine/6840ptm.h"
#include "machine/clock.h"
#include "bus/rs232/rs232.h"

#include "ibm5085.lh"

namespace {

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
		m_test_switch(*this, "SWITCHES")
	{
    }

	void ibm50852a(machine_config &config) ATTR_COLD;

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

private:
	// Finders
	required_device<cpu_device> m_att_cpu;
	required_region_ptr<u16> m_att_rom;
	memory_view m_bootvect;
	output_finder<2> m_diag_leds;
	output_finder<> m_mcheck_led;
	required_device<i8274_device> m_periph12_uart;
	required_ioport m_test_switch;

	emu_timer *m_boot_timer = nullptr;
	TIMER_CALLBACK_MEMBER(boot_timeout);

	void att_map(address_map &map) ATTR_COLD;
	void diag_led_w(u8 data);
	u8 testsw_r();
	u16 ext_bus_r(offs_t offset);
};

void ibm5085_state::machine_start()
{
	m_boot_timer = timer_alloc(FUNC(ibm5085_state::boot_timeout), this);
}

void ibm5085_state::machine_reset()
{
	m_bootvect.select(0);
	m_boot_timer->adjust(attotime::from_usec(4));
}

void ibm5085_state::att_map(address_map &map)
{
	map(0x000000, 0x7ffff).ram().share("ram_private");
	map(0xfd0000, 0xfd7fff).rom().mirror(0x008000).region("att_rom", 0);
	map(0xfe0000, 0xfe7fff).rom().mirror(0x008000).region("att_rom", 0);

	map(0xfff002, 0xfff002).w(FUNC(ibm5085_state::diag_led_w)).umask16(0xff00);
	map(0xfff005, 0xfff005).r(FUNC(ibm5085_state::testsw_r)).umask16(0x00ff);
	map(0xfff00c, 0xfff00f).r(FUNC(ibm5085_state::ext_bus_r));

	map(0xff0040, 0xff0047).rw(m_periph12_uart, FUNC(i8274_device::ba_cd_r), FUNC(i8274_device::ba_cd_w)).umask16(0x00ff);

	map(0x000000, 0x000007).view(m_bootvect);
	m_bootvect[0](0x000000, 0x000007).rom().region("att_rom", 0);
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

u16 ibm5085_state::ext_bus_r(offs_t offset)
{
	if (offset == 0) {
		return 0xff00;
	} else {
		return 0xffff;
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
