// license:BSD-3-Clause
// copyright-holders:Chris Simmons

/*
 * Western Digital WD1933/WD1935 Synchronous Data Link Controller.
 *
 * Sources:
 *  - Western Digital WD1933 Synchronous Data Link Controller data sheet
 *
 * This is a frame-level model: characters are transferred to and from the
 * line at the character rate derived from the device clock (taken to be
 * the 1X bit rate), but bit-level details such as zero insertion, NRZI,
 * residual characters, non-8-bit characters, and the DPLL are not modelled.
 *
 * The FCS is generated and checked, and (as on the real device) the two
 * received FCS characters are transferred through the receiver holding
 * register to the host.
 *
 * TODO:
 *  - WD1935 differences
 *  - sdlc loop mode
 *  - extended address/control
 *  - residual and short characters
 *  - CD/RI interrupting edge selection inputs
 */

#include "emu.h"
#include "wd1933.h"

#define LOG_REG   (1U << 1)
#define LOG_TX    (1U << 2)
#define LOG_RX    (1U << 3)
#define LOG_INT   (1U << 4)

//#define VERBOSE (LOG_GENERAL|LOG_REG|LOG_TX|LOG_RX|LOG_INT)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(WD1933, wd1933_device, "wd1933", "Western Digital WD1933 Synchronous Data Link Controller")
DEFINE_DEVICE_TYPE(WD1935, wd1935_device, "wd1935", "Western Digital WD1935 Synchronous Data Link Controller")

wd1933_device::wd1933_device(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, type, tag, owner, clock)
	, m_out_intrq(*this)
	, m_out_drqo(*this)
	, m_out_drqi(*this)
	, m_out_rts(*this)
	, m_out_dtr(*this)
	, m_out_misc(*this)
	, m_out_line(*this)
	, m_frame_tx(*this)
	, m_tx_timer(nullptr)
	, m_rx_timer(nullptr)
	, m_eob(false)
	, m_dsr(false)
	, m_cd(false)
	, m_ri(false)
	, m_misc(false)
{
}

wd1933_device::wd1933_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: wd1933_device(mconfig, WD1933, tag, owner, clock)
{
}

wd1935_device::wd1935_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: wd1933_device(mconfig, WD1935, tag, owner, clock)
{
}

void wd1933_device::device_start()
{
	m_frame_tx.resolve_safe();

	m_tx_timer = timer_alloc(FUNC(wd1933_device::tx_tick), this);
	m_rx_timer = timer_alloc(FUNC(wd1933_device::rx_tick), this);

	save_item(NAME(m_cr1));
	save_item(NAME(m_cr2));
	save_item(NAME(m_cr3));
	save_item(NAME(m_ar));
	save_item(NAME(m_rhr));
	save_item(NAME(m_thr));
	save_item(NAME(m_ir));
	save_item(NAME(m_sr));
	save_item(NAME(m_intrq));
	save_item(NAME(m_drqo));
	save_item(NAME(m_drqi));
	save_item(NAME(m_eob));
	save_item(NAME(m_dsr));
	save_item(NAME(m_cd));
	save_item(NAME(m_ri));
	save_item(NAME(m_misc));
	save_item(NAME(m_tx_state));
	save_item(NAME(m_thr_full));
	save_item(NAME(m_tx_cmd));
	save_item(NAME(m_tx_fcs));
	save_item(NAME(m_tx_crc));
	save_item(NAME(m_rx_state));
	save_item(NAME(m_rx_ovr));
	save_item(NAME(m_rx_count));
	save_item(NAME(m_rx_crc));
}

void wd1933_device::device_reset()
{
	m_cr1 = 0;
	m_cr2 = 0;
	m_cr3 = 0;
	m_ar = 0;
	m_rhr = 0;
	m_thr = 0;
	m_ir = 0;
	m_sr = 0;

	m_intrq = false;
	m_drqo = false;
	m_drqi = false;

	m_tx_state = TX_IDLE;
	m_thr_full = false;
	m_tx_cmd = false;
	m_tx_fcs = 0;
	m_tx_crc = 0xffff;
	m_tx_frame.clear();

	m_rx_state = RX_HUNT;
	m_rx_ovr = false;
	m_rx_count = 0;
	m_rx_crc = 0xffff;
	m_rx_queue.clear();

	m_tx_timer->adjust(attotime::never);
	m_rx_timer->adjust(attotime::never);

	m_out_intrq(0);
	m_out_drqo(0);
	m_out_drqi(0);
	m_out_rts(0);
	m_out_dtr(0);
	m_out_misc(0);
}

u8 wd1933_device::read(offs_t offset)
{
	u8 data = 0;

	switch (offset & 7)
	{
	case 7: data = m_cr1; break;
	case 6: data = m_cr2; break;
	case 5: data = m_cr3; break;
	case 4: // receiver holding register
		data = m_rhr;
		if (!machine().side_effects_disabled())
			set_drqi(false);
		break;
	case 3: // interrupt register
		data = m_ir | (m_drqi ? IR_DRQI : 0) | (m_drqo ? IR_DRQO : 0) | (m_intrq ? IR_INTRQ : 0);
		if (!machine().side_effects_disabled())
		{
			m_ir = 0;
			update_intrq();
		}
		break;
	case 2: // status register
		data = m_sr & (SR_CRC | SR_OVR | SR_ABT);
		if (m_rx_state == RX_HUNT)
			data |= SR_IDLE;
		if (self_test())
			data |= SR_DSR | SR_CD;
		else
			data |= (m_misc ? SR_MISC : 0) | (m_dsr ? SR_DSR : 0) | (m_cd ? SR_CD : 0) | (m_ri ? SR_RI : 0);
		if (!machine().side_effects_disabled())
			m_sr &= ~(SR_CRC | SR_OVR | SR_ABT);
		break;
	default:
		break;
	}

	LOGMASKED(LOG_REG, "read %u data 0x%02x (%s)\n", offset & 7, data, machine().describe_context());

	return data;
}

void wd1933_device::write(offs_t offset, u8 data)
{
	LOGMASKED(LOG_REG, "write %u data 0x%02x (%s)\n", offset & 7, data, machine().describe_context());

	switch (offset & 7)
	{
	case 7: cr1_w(data); break;
	case 6: cr2_w(data); break;
	case 5: m_cr3 = data & 0x07; break;
	case 4: m_ar = data; break;
	case 3: thr_w(data); break;
	default:
		break;
	}
}

void wd1933_device::cr1_w(u8 data)
{
	u8 const changed = m_cr1 ^ data;
	m_cr1 = data;

	if (changed & CR1_MISC)
		m_out_misc(BIT(data, 0));
	if (changed & CR1_DTR)
		m_out_dtr(BIT(data, 1) && !self_test());

	// receiver
	if ((changed & CR1_ACTR) && !(data & CR1_ACTR))
	{
		// deactivating the receiver discards any unread character
		LOGMASKED(LOG_RX, "%s receiver deactivated\n", machine().time().as_string(6));
		m_rx_state = RX_HUNT;
		m_rx_ovr = false;
		set_drqi(false);
	}

	// transmitter
	if (changed & CR1_ACTT)
	{
		m_out_rts(BIT(data, 6));

		if (data & CR1_ACTT)
		{
			LOGMASKED(LOG_TX, "transmitter activated command %u\n", tc());
			m_tx_timer->adjust(char_time(), 0, char_time());
		}
		else
		{
			LOGMASKED(LOG_TX, "transmitter deactivated\n");
			if (m_tx_state == TX_FRAME)
				tx_abort();
			m_tx_state = TX_IDLE;

			// deactivating the transmitter discards any pending character
			m_thr_full = false;
			m_eob = false;
			m_tx_timer->adjust(attotime::never);
			m_tx_cmd = false;
			set_drqo(false);
			return;
		}
	}

	if (data & CR1_ACTT)
	{
		if (tc() == TC_DATA)
		{
			m_tx_cmd = false;
			if (m_tx_state != TX_FCS)
				set_drqo(!m_thr_full);
		}
		else if ((changed & (CR1_TC | CR1_ACTT)))
		{
			// abort, flag, or fcs command
			m_tx_cmd = true;
			set_drqo(false);
		}
	}
}

void wd1933_device::cr2_w(u8 data)
{
	u8 const changed = m_cr2 ^ data;
	m_cr2 = data;

	if (changed & CR2_ST)
	{
		LOGMASKED(LOG_RX, "self-test %s\n", (data & CR2_ST) ? "on" : "off");

		// breaking the loop while a looped frame is being received aborts it
		if (m_rx_state == RX_FRAME && !(data & CR2_ST))
			rx_event(RX_ABORT);
		else if (data & CR2_ST)
			m_rx_queue.clear();

		m_out_dtr(BIT(m_cr1, 1) && !self_test());
		modem_change();
	}
}

void wd1933_device::thr_w(u8 data)
{
	LOGMASKED(LOG_REG, "thr 0x%02x\n", data);

	m_thr = data;
	m_thr_full = true;
	set_drqo(false);
}

void wd1933_device::eob_w(int state)
{
	if (state)
	{
		LOGMASKED(LOG_TX, "eob asserted\n");
		m_eob = true;
	}
}

void wd1933_device::dsr_w(int state)
{
	if (m_dsr != bool(state))
	{
		m_dsr = state;
		modem_change();
	}
}

void wd1933_device::cd_w(int state)
{
	if (m_cd != bool(state))
	{
		m_cd = state;
		modem_change();
	}
}

void wd1933_device::ri_w(int state)
{
	if (m_ri != bool(state))
	{
		m_ri = state;
		modem_change();
	}
}

void wd1933_device::misc_w(int state)
{
	m_misc = state;
}

void wd1933_device::modem_change()
{
	if (!self_test())
		set_ir(IR_DSC);
}

void wd1933_device::set_drqo(bool state)
{
	if (m_drqo != state)
	{
		m_drqo = state;
		m_out_drqo(state);
	}
}

void wd1933_device::set_drqi(bool state)
{
	if (m_drqi != state)
	{
		m_drqi = state;
		m_out_drqi(state);
	}
}

void wd1933_device::set_ir(u8 mask)
{
	LOGMASKED(LOG_INT, "interrupt 0x%02x\n", mask);

	m_ir |= mask;
	update_intrq();
}

void wd1933_device::update_intrq()
{
	bool const intrq = m_ir & (IR_DSC | IR_XUND | IR_XOK | IR_REOME | IR_REOM);

	if (m_intrq != intrq)
	{
		m_intrq = intrq;
		m_out_intrq(intrq);
	}
}

u16 wd1933_device::crc_update(u16 crc, u8 data)
{
	// CRC-CCITT, bit-reversed (transmitted least significant bit first)
	crc ^= data;
	for (unsigned i = 0; i < 8; i++)
		crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);

	return crc;
}

TIMER_CALLBACK_MEMBER(wd1933_device::tx_tick)
{
	switch (m_tx_state)
	{
	case TX_IDLE:
		if (tc() == TC_DATA)
		{
			if (m_thr_full)
			{
				LOGMASKED(LOG_TX, "frame start\n");
				m_tx_state = TX_FRAME;
				m_tx_frame.clear();
				m_tx_crc = 0xffff;
				tx_shift();
			}
		}
		else if (m_tx_cmd)
		{
			// flag, abort, or fcs command completed outside a frame
			m_tx_cmd = false;
			set_ir(IR_XOK);
		}
		break;

	case TX_FRAME:
		if (m_thr_full && tc() == TC_DATA)
			tx_shift();
		else if (m_eob || tc() == TC_FCS)
		{
			LOGMASKED(LOG_TX, "fcs (%s)\n", m_eob ? "eob" : "command");
			m_tx_state = TX_FCS;
			m_tx_fcs = 3;
			m_tx_crc = ~m_tx_crc;
			set_drqo(false);

			// first fcs character
			tx_line(RX_BYTE | (m_tx_crc & 0xff));
			m_tx_fcs--;
		}
		else if (tc() != TC_DATA)
		{
			// abort or flag command terminates the frame without fcs
			tx_abort();
			m_tx_state = TX_IDLE;
			m_tx_cmd = false;
			set_ir(IR_XOK);
		}
		else
		{
			LOGMASKED(LOG_TX, "underrun\n");
			tx_abort();
			m_tx_state = TX_IDLE;
			set_drqo(false);
			set_ir(IR_XUND);
		}
		break;

	case TX_FCS:
		if (m_tx_fcs == 2)
			tx_line(RX_BYTE | (m_tx_crc >> 8));
		else if (m_tx_fcs == 1)
		{
			// closing flag
			LOGMASKED(LOG_TX, "%s frame complete length %u\n", machine().time().as_string(6), m_tx_frame.size());
			tx_line(RX_END);
			if (!self_test())
				m_frame_tx(m_tx_frame);

			m_tx_frame.clear();
			m_tx_state = TX_IDLE;
			m_eob = false;
			m_tx_cmd = false;
			set_ir(IR_XOK);

			if (tc() == TC_DATA)
				set_drqo(!m_thr_full);
		}
		m_tx_fcs--;
		break;
	}
}

void wd1933_device::tx_shift()
{
	u8 const data = m_thr;

	m_thr_full = false;
	m_tx_frame.push_back(data);
	m_tx_crc = crc_update(m_tx_crc, data);

	tx_line(RX_BYTE | data);

	// request the next character
	set_drqo(true);
}

void wd1933_device::tx_line(u16 event)
{
	// in self-test mode, the transmitter output is looped to the receiver
	if (self_test())
		rx_event(event);
	else
		m_out_line(event);
}

void wd1933_device::line_w(u16 event)
{
	if (!self_test())
		rx_event(event);
}

void wd1933_device::tx_abort()
{
	LOGMASKED(LOG_TX, "frame aborted\n");

	tx_line(RX_ABORT);
	m_tx_frame.clear();
	m_eob = false;
}

void wd1933_device::rx_frame(u8 const *data, unsigned length)
{
	if (self_test())
		return;

	u16 crc = 0xffff;
	for (unsigned i = 0; i < length; i++)
	{
		m_rx_queue.push_back(RX_BYTE | data[i]);
		crc = crc_update(crc, data[i]);
	}

	crc = ~crc;
	m_rx_queue.push_back(RX_BYTE | (crc & 0xff));
	m_rx_queue.push_back(RX_BYTE | (crc >> 8));
	m_rx_queue.push_back(RX_END);

	if (m_rx_timer->expire().is_never())
		m_rx_timer->adjust(char_time());
}

TIMER_CALLBACK_MEMBER(wd1933_device::rx_tick)
{
	if (!m_rx_queue.empty())
	{
		rx_event(m_rx_queue.front());
		m_rx_queue.pop_front();
	}

	m_rx_timer->adjust(m_rx_queue.empty() ? attotime::never : char_time());
}

void wd1933_device::rx_event(u16 event)
{
	if (!(m_cr1 & CR1_ACTR))
	{
		LOGMASKED(LOG_RX, "%s receiver inactive, event 0x%03x ignored\n", machine().time().as_string(6), event);
		m_rx_state = RX_HUNT;
		return;
	}

	switch (event & 0x300)
	{
	case RX_BYTE:
		{
			u8 const data = event & 0xff;

			if (m_rx_state == RX_HUNT)
			{
				if ((m_cr2 & CR2_AC) && data != m_ar && data != 0xff)
				{
					LOGMASKED(LOG_RX, "frame address 0x%02x discarded\n", data);
					m_rx_state = RX_DISCARD;
					break;
				}

				LOGMASKED(LOG_RX, "%s frame start address 0x%02x\n", machine().time().as_string(6), data);
				m_rx_state = RX_FRAME;
				m_rx_ovr = false;
				m_rx_count = 0;
				m_rx_crc = 0xffff;
			}

			if (m_rx_state == RX_FRAME)
			{
				if (m_drqi)
				{
					LOGMASKED(LOG_RX, "overrun\n");
					m_rx_ovr = true;
				}

				m_rhr = data;
				m_rx_count++;
				m_rx_crc = crc_update(m_rx_crc, data);
				set_drqi(true);
			}
		}
		break;

	case RX_END:
		if (m_rx_state == RX_FRAME)
		{
			u8 error = 0;

			if (m_rx_count < 4)
				error = SR_ABT;
			else if (m_rx_ovr)
				error = SR_OVR;
			else if (m_rx_crc != 0xf0b8)
				error = SR_CRC;

			LOGMASKED(LOG_RX, "frame end length %u status 0x%02x\n", m_rx_count, error);

			m_sr = (m_sr & ~(SR_CRC | SR_OVR | SR_ABT)) | error;
			set_ir(error ? IR_REOME : IR_REOM);
		}
		m_rx_state = RX_HUNT;
		break;

	case RX_ABORT:
		if (m_rx_state == RX_FRAME)
		{
			LOGMASKED(LOG_RX, "frame aborted\n");

			m_sr = (m_sr & ~(SR_CRC | SR_OVR | SR_ABT)) | SR_ABT;
			set_ir(IR_REOME);
		}
		m_rx_state = RX_HUNT;
		break;
	}
}
