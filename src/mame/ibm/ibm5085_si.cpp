// license:BSD-3-Clause
// copyright-holders:Christopher Simmons

#include "emu.h"
#include "ibm5085_si.h"

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(IBM5085_SI, ibm5085_si_device, "ibm5085_si", "IBM 5085 SI Board")

ibm5085_si_device::ibm5085_si_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, IBM5085_SI, tag, owner, clock)
	, m_cpu(*this, "cpu")       // tags are relative to this device: full path is ":<device tag>:cpu"
	, m_rom(*this, "rom")
	, m_sram(*this, "sram")
	, m_page0_bank(*this, "page0_bank")
	, m_ff8_bank(*this, "ff8_bank")
    , m_bootvect(*this, "bootvect")
	, m_fdc(*this, "fdc")
	, m_floppy(*this, "fdc:0")
	, m_int_cb(*this)
{
}

static void si_floppies(device_slot_interface &device)
{	
	device.option_add("35dd", FLOPPY_35_DD);
}

void ibm5085_si_device::device_add_mconfig(machine_config &config)
{
	M68010(config, m_cpu, 8'000'000);   // PLACEHOLDER clock
	m_cpu->set_addrmap(AS_PROGRAM, &ibm5085_si_device::cpu_map);

	UPD765A(config, m_fdc, 8'000'000, true, true);
	m_fdc->intrq_wr_callback().set(FUNC(ibm5085_si_device::fdc_int_w));
	FLOPPY_CONNECTOR(config, m_floppy, si_floppies, "35dd", floppy_image_device::default_pc_floppy_formats);
}

void ibm5085_si_device::cpu_map(address_map &map)
{
    map(0xfcf000, 0xfcf001).w(FUNC(ibm5085_si_device::doorbell_w)).umask16(0x00ff);

	map(0xff0000, 0xff3fff).rom().region("rom", 0);

	map(0xffc000, 0xffffff).ram().share(m_sram);
	map(0xff8000, 0xff8fff).bankrw(m_ff8_bank);
	map(0x000000, 0x000fff).bankrw(m_page0_bank);

    map(0x000000, 0x000007).view(m_bootvect);
	m_bootvect[0](0x000000, 0x000007).rom().region("rom", 0);
}

void ibm5085_si_device::host_map(address_map &map)
{
	map(0x000, 0x001).w(FUNC(ibm5085_si_device::doorbell_w)).umask16(0x00ff);
	map(0xe00, 0xe00).rw(m_fdc, FUNC(upd765a_device::fifo_r), FUNC(upd765a_device::fifo_w));
	map(0xe01, 0xe01).r(m_fdc, FUNC(upd765a_device::msr_r));
	map(0xe06, 0xe06).w(FUNC(ibm5085_si_device::motor_w));
}

void ibm5085_si_device::device_start()
{
	m_page0_bank->set_base(&m_sram[0x3000 / 2]);
	m_ff8_bank->set_base(&m_sram[0x0000 / 2]);

    m_boot_timer = timer_alloc(FUNC(ibm5085_si_device::boot_timeout), this);

    // HACK: MAME's 68010 doesn't implement instruction continuation when returning
	// from a format $8 frame. The CPU check at FF35EE relies on it: its 68010 path
	// rewrites the fault address so the rerun of tst.w $1 succeeds. Make that path
	// skip the faulting instruction instead (clr.l ($a,a7) -> addq.l #4,($2,a7)).
	if (m_rom[0x3616 / 2] == 0x42af && m_rom[0x3618 / 2] == 0x000a)
	{
		m_rom[0x3616 / 2] = 0x58af;
		m_rom[0x3618 / 2] = 0x0002;
        m_rom[0x3ffe / 2] = 0x93ab;     // New checksum
	}
}

void ibm5085_si_device::device_reset()
{
    m_bootvect.select(0);
	m_boot_timer->adjust(attotime::from_usec(4));
}

void ibm5085_si_device::doorbell_w(u8 data)
{
    if (!BIT(data, 0))
    {
        LOG("Set IRQ4\n");
        m_cpu->set_input_line(M68K_IRQ_4, ASSERT_LINE);
    } else
    {
        LOG("Clear IRQ4\n");
        m_cpu->set_input_line(M68K_IRQ_4, CLEAR_LINE);
    }
}

void ibm5085_si_device::motor_w(u8 data)
{
	if (floppy_image_device *f = m_floppy->get_device())
		f->mon_w(!BIT(data, 0));
}

void ibm5085_si_device::fdc_int_w(int state)
{
	m_int_cb(state);
}

TIMER_CALLBACK_MEMBER(ibm5085_si_device::boot_timeout)
{
	m_bootvect.disable();
}

ROM_START(ibm5085_si)
	ROM_REGION16_BE(0x4000, "rom", 0)
	ROM_LOAD16_BYTE("1915517p099.bin", 0x0000, 0x2000, CRC(f13a9475) SHA1(01a0e50f9cac3592d5d3a701d208fb7a9672a380))
	ROM_LOAD16_BYTE("1915517p098.bin", 0x0001, 0x2000, CRC(d646a1f0) SHA1(a6d97cbae98a0d4e81293c65c51349289285da0a))
ROM_END

tiny_rom_entry const *ibm5085_si_device::device_rom_region() const
{
	return ROM_NAME(ibm5085_si);
}
