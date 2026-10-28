
#include "emu.h"
#include "ibm5085_pca.h"
#include "rtpc_kbd.h"

#include <bit>

#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(IBM5085_PCA, ibm5085_pca_device, "ibm5085_pca", "IBM 5085 Peripheral Connector Assembly")

ibm5085_pca_device::ibm5085_pca_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: buffered_rs232_device(mconfig, IBM5085_PCA, tag, owner, clock)
	, m_kbdc(*this, "kbdc")
	, m_rts_timer(nullptr)
	, m_state(IDLE)
	, m_shift(0)
	, m_bit(0)
	, m_loopback(false)
	, m_config(0x80)
{
}

static void pca_kbd_devices(device_slot_interface &device)
{
	device.option_add("keyboard", RTPC_KBD);
}

void ibm5085_pca_device::device_add_mconfig(machine_config &config)
{
	RTPC_KBDC(config, m_kbdc, pca_kbd_devices, "keyboard");
	m_kbdc->out_clock_cb().set(FUNC(ibm5085_pca_device::kbd_clock_w));
}

void ibm5085_pca_device::device_start()
{
    m_rts_timer = timer_alloc(FUNC(ibm5085_pca_device::kbd_request_to_send), this);

    save_item(NAME(m_state));
    save_item(NAME(m_shift));
    save_item(NAME(m_bit));
    save_item(NAME(m_loopback));
    save_item(NAME(m_config));
}

void ibm5085_pca_device::device_reset()
{
	set_data_frame(1, 8, PARITY_ODD, STOP_BITS_1);
	set_rate(9600);
	output_rxd(1);
	output_cts(0);
	output_dcd(0);

	m_loopback = false;
	m_config = 0x80;
	m_state = IDLE;
}

void ibm5085_pca_device::received_byte(u8 data)
{
	if (m_loopback)
	{
		if (data == 0x0f) m_loopback = false;   // your script: 0x0F also resets config
		else transmit_byte(data);
		return;
	}
	switch (data)
	{
	case 0x0a: transmit_byte(m_config); break;
	case 0x0c: transmit_byte(0x55); transmit_byte(0xaa); break;
	case 0x0e: m_loopback = true; break;
	// ... the rest of your script's internal commands
	default:
		if (data >= 0xe0) kbd_send(data);
		break;
	}
}

// Host -> keyboard: pull clock low to inhibit, then signal a request to send
void ibm5085_pca_device::kbd_send(u8 data)
{
	if (m_state != IDLE)
	{
		LOG("kbd_send 0x%02x dropped, link busy\n", data);
		return;
	}

	// Frame after the start bit: D0-D7, odd parity, stop (1)
	bool const parity = !(std::popcount(data) & 1);
	m_shift = data | (parity << 8) | (1 << 9);
	m_bit = 0;

	m_state = TX_HOLD;
	m_kbdc->clock_write_from_mb(0);
	m_rts_timer->adjust(attotime::from_usec(100));
}

TIMER_CALLBACK_MEMBER(ibm5085_pca_device::kbd_request_to_send)
{
	// Start bit, then hand the clock to the keyboard
	m_kbdc->data_write_from_mb(0);
	m_state = TX;
	m_kbdc->clock_write_from_mb(1);
}

void ibm5085_pca_device::kbd_clock_w(int state)
{
	// The keyboard samples on the rising edge; the host changes data and samples on the falling edge
	if (state)
		return;

	switch (m_state)
	{
	case IDLE:
		// Keyboard start bit
		if (!m_kbdc->data_signal())
		{
			m_state = RX;
			m_shift = 0;
			m_bit = 0;
		}
		break;

	case RX:
		// Bits 0-7 data, 8 parity, 9 stop
		m_shift |= m_kbdc->data_signal() << m_bit;
		if (++m_bit == 10)
		{
			m_state = IDLE;
			u8 const data = m_shift & 0xff;
			if (BIT(m_shift, 8) == BIT(std::popcount(data), 0))
				LOG("kbd rx parity error 0x%03x\n", m_shift);
			if (!BIT(m_shift, 9))
				LOG("kbd rx framing error 0x%03x\n", m_shift);
			kbd_received(data);
		}
		break;

	case TX_HOLD:
		// Our own clock_write_from_mb(0) echoes back here; ignore it
		break;

	case TX:
		if (m_bit < 10)
		{
			// Edges 1-10: D0-D7, parity, then release data for the stop bit
			m_kbdc->data_write_from_mb(BIT(m_shift, m_bit));
			m_bit++;
		}
		else
		{
			// Edge 11: keyboard pulls data low to acknowledge
			if (m_kbdc->data_signal())
				LOG("kbd tx no ack\n");
			m_state = IDLE;
		}
		break;
	}
}

// Keyboard -> host: the PCA passes scan codes and command replies through
void ibm5085_pca_device::kbd_received(u8 data)
{
	LOG("kbd -> AP 0x%02x\n", data);
	transmit_byte(data);
}