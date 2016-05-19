#include "ledtorus.h"
#include "nrf24l01p.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>


/* Communications protocol. */
#define POV_CMD_DEBUG 254
#define POV_SUBCMD_RESET_TO_BOOTLOADER 255
#define POV_SUBCMD_ENTER_BOOTLOADER 254
#define POV_SUBCMD_RESET_TO_APP 253
#define POV_SUBCMD_FLASH_BUFFER 252
#define POV_SUBCMD_EXIT_DEBUG   251
#define POV_SUBCMD_STATUS_REPLY 240

#define POV_CMD_CONFIG 255
#define POV_SUBCMD_SET_CONFIG 1
#define POV_SUBCMD_KEYPRESSES 2


/*
  State of pressed keys. Keys are active high ("1" bit means key is pressed).

  Byte 0: Special keys for LED-torus (mirrored by some DualShock keys):

    bit 0  button 1 / Left     (used to go to next animation)
        1  button 2 / Right    (used to go to previous animation)
        2  button 3 / Down     (used to decrease LED intensity)
        3  button 4 / Up       (used to increase LED intensity)
        4  button 5 / Select
        5  switch 1            (used to select manual/auto mode)
        6  switch 2
        7  switch 3            (used to start/stop motor)

  Byte 1: DualShock keys

    bit 0: SELECT
    bit 1: L3
    bit 2  R3
    bit 3  START
    bit 4  Up
    bit 5  Right
    bit 6  Down
    bit 7  Left

   Byte 2: More DualShock keys
    bit 0: L2
    bit 1: R2
    bit 2  L1
    bit 3  R1
    bit 4  Triangle
    bit 5  Circle
    bit 6  Cross
    bit 7  Square

   Byte 3: Joystick R left (0x00) -> right (0xff)
   Byte 4: Joystick R up (0x00) -> down (0xff)
   Byte 5: Joystick L left (0x00) -> right (0xff)
   Byte 6: Joystick L up (0x00) -> down (0xff)

   Byte 7-18: Pressure-sensitivity values (0..0xff) for the DualShock keys:
     7: Button "right" pressure sensitivity
     8: Button "left" pressure sensitivity
     9: Button "up" pressure sensitivity
     10: Button "down" pressure sensitivity
     11: Button "triangle" pressure sensitivity
     12: Button "circle" pressure sensitivity
     13: Button "cross" pressure sensitivity
     14: Button "square" pressure sensitivity
     15: Button "L1" pressure sensitivity
     16: Button "R1" pressure sensitivity
     17: Button "L2" pressure sensitivity
     18: Button "R2" pressure sensitivity
*/

uint8_t volatile key_state[19];
#define MAX_KEY_EVENTS 16

static volatile uint32_t key_events[MAX_KEY_EVENTS];
static volatile uint32_t key_event_head = 0, key_event_tail = 0;


static void
nrf_irq_enable(void)
{
  EXTI_IMR |= EXTI3;
}


static void
nrf_irq_disable(void)
{
  EXTI_IMR &= ~EXTI3;
}


static void
nrf_irq_clear(void)
{
  EXTI_PR = EXTI3;
}


static uint32_t
nrf_irq_pinstatus(void)
{
  return GPIOC_IDR & GPIO3;
}


/*
  Setup SPI communication for nRF42L01+ on USART1 in synchronous mode.

    PA8   clk
    PB6   mosi
    PB7   miso
    PC1   cs
    PC2   ce
    PC3   irq
*/
static void
setup_nrf_spi(void)
{
  uint32_t tmp;

  rcc_periph_clock_enable(RCC_USART1);
  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_GPIOC);

  usart_disable(USART1);

  /*
    Clock on PA8.
    Polarity is idle low, active high.
    Phase is sample on rising, setup on falling edge.
  */
  gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLDOWN, GPIO8);
  gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO8);
  gpio_set_af(GPIOA, GPIO_AF7, GPIO8);

  /* MOSI and MISO on PB6/PB7. */
  gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6|GPIO7);
  gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO6|GPIO7);
  gpio_set_af(GPIOB, GPIO_AF7, GPIO6|GPIO7);

  /*
    CS on PC1, CE on PC2.
    CS is high initially (active low).
    CE is low initially (active high).
  */
  gpio_set(GPIOC, GPIO1);
  gpio_clear(GPIOC, GPIO2);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1|GPIO2);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO1|GPIO2);

  usart_set_baudrate(USART1, 5250000);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  /* ToDo: Maybe do an API for USART SPI mode. */
  tmp = USART1_CR2;
  tmp &= ~(USART_CR2_CLKEN|USART_CR2_CPOL|USART_CR2_CPHA|USART_CR2_LBCL);
  /* Zero values for CPOL/CPHA gives CPOL_log and CPHA_1Edge. */
  tmp |= USART_CR2_CLKEN|USART_CR2_LBCL;
  USART1_CR2 = tmp;

  usart_enable(USART1);

  /* Setup DMA. USART1 on DMA2 channel 4, streams 2 (Rx) and 7 (Tx). */
  rcc_periph_clock_enable(RCC_DMA2);
  dma_stream_reset(DMA2, DMA_STREAM2);
  dma_stream_reset(DMA2, DMA_STREAM7);

  dma_enable_direct_mode(DMA2, DMA_STREAM7);
  dma_set_fifo_threshold(DMA2, DMA_STREAM7, DMA_SxFCR_FTH_1_4_FULL);
  dma_set_memory_burst(DMA2, DMA_STREAM7, DMA_SxCR_MBURST_SINGLE);
  dma_set_peripheral_burst(DMA2, DMA_STREAM7, DMA_SxCR_PBURST_SINGLE);
  dma_set_memory_size(DMA2, DMA_STREAM7, DMA_SxCR_MSIZE_8BIT);
  dma_set_peripheral_size(DMA2, DMA_STREAM7, DMA_SxCR_PSIZE_8BIT);
  dma_enable_memory_increment_mode(DMA2, DMA_STREAM7);
  dma_disable_peripheral_increment_mode(DMA2, DMA_STREAM7);
  dma_set_priority(DMA2, DMA_STREAM7, DMA_SxCR_PL_MEDIUM);
  dma_set_peripheral_address(DMA2, DMA_STREAM7, (uint32_t) (&(USART1_DR)));
  dma_channel_select(DMA2, DMA_STREAM7, DMA_SxCR_CHSEL_4);
  dma_set_transfer_mode(DMA2, DMA_STREAM7, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

  dma_enable_direct_mode(DMA2, DMA_STREAM2);
  dma_set_fifo_threshold(DMA2, DMA_STREAM2, DMA_SxFCR_FTH_1_4_FULL);
  dma_set_memory_burst(DMA2, DMA_STREAM2, DMA_SxCR_MBURST_SINGLE);
  dma_set_peripheral_burst(DMA2, DMA_STREAM2, DMA_SxCR_PBURST_SINGLE);
  dma_set_memory_size(DMA2, DMA_STREAM2, DMA_SxCR_MSIZE_8BIT);
  dma_set_peripheral_size(DMA2, DMA_STREAM2, DMA_SxCR_PSIZE_8BIT);
  dma_enable_memory_increment_mode(DMA2, DMA_STREAM2);
  dma_disable_peripheral_increment_mode(DMA2, DMA_STREAM2);
  dma_set_priority(DMA2, DMA_STREAM2, DMA_SxCR_PL_MEDIUM);
  dma_set_peripheral_address(DMA2, DMA_STREAM2, (uint32_t) (&(USART1_DR)));
  dma_channel_select(DMA2, DMA_STREAM2, DMA_SxCR_CHSEL_4);
  dma_set_transfer_mode(DMA2, DMA_STREAM2, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);

  /* Configure a USART1 DMA Rx transfer complete interrupt. */
  dma_disable_transfer_complete_interrupt(DMA2, DMA_STREAM2);
  nvic_set_priority(NVIC_DMA2_STREAM2_IRQ, 10<<4);
  nvic_enable_irq(NVIC_DMA2_STREAM2_IRQ);
  dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM2);

  /* IRQ on PC3. */
  gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO3);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO3);

  /* We only want to configure the interrupt here, not yet enable it. */
  rcc_periph_clock_enable(RCC_SYSCFG);
  exti_disable_request(EXTI3);
  /* Take an interrupt on falling edge (IRQ is active low). */
  exti_select_source(EXTI3, GPIOC);
  exti_set_trigger(EXTI3, EXTI_TRIGGER_FALLING);

  /* Clear any pending interrupt before enabling in NVIC. */
  nrf_irq_clear();
  nvic_set_priority(NVIC_EXTI3_IRQ, 10<<4);
  nvic_enable_irq(NVIC_EXTI3_IRQ);
}


static inline void
csn_low(void)
{
  gpio_clear(GPIOC, GPIO1);
}


static inline void
csn_high(void)
{
  gpio_set(GPIOC, GPIO1);
}


static inline void
ce_low(void)
{
  gpio_clear(GPIOC, GPIO2);
}


static inline void
ce_high(void)
{
  gpio_set(GPIOC, GPIO2);
}


static inline uint8_t
bitswap_byte(uint8_t in)
{
  return (uint8_t)(asm_rbit((uint32_t)in) >> 24);
}


static uint8_t nrf_send_buffer[33];
static uint8_t nrf_recv_buffer[33];
static volatile uint8_t nrf_cmd_running = 0;
static volatile uint8_t receive_multi_running = 0;
static volatile uint8_t nrf_idle = 0;

/*
  This function starts DMA to issue an nRF24L01+ command.
  The DMA transfer will run in the background. Interrupts should be used to
  track completion.
  When DMA Rx completion interrupt runs, call ssi_cmd_transfer_done() to
  complete the command.
*/
static void
ssi_cmd_start(uint32_t len)
{
  /* Take CSN low to initiate transfer. */
  csn_low();

  DMA2_S2M0AR = &nrf_recv_buffer[0];
  DMA2_S2NDTR = len;
  DMA2_S7M0AR = &nrf_send_buffer[0];
  DMA2_S7NDTR = len;
  /* Clear DMA transfer complete flags. */
  DMA2_LIFCR = DMA_LISR_TCIF2;
  DMA2_HIFCR = DMA_HISR_TCIF7;
  /* Clear the  USART TC (transfer complete) flag. */
  USART1_SR &= ~USART_SR_TC;
  /* Enable the Rx and Tx DMA channels. */
  DMA2_S2CR |= DMA_SxCR_EN;
  DMA2_S7CR |= DMA_SxCR_EN;
  /* Enable the USART1 to generate Rx/Tx DMA requests. */
  USART1_CR3 |= (USART_CR3_DMAT|USART_CR3_DMAR);
}


static void
ssi_cmd_transfer_done()
{
  /* Take CSN high to complete transfer. */
  csn_high();

  /* Disable DMA requests and channels. */
  DMA2_S2CR &= ~DMA_SxCR_EN;
  DMA2_S7CR &= ~DMA_SxCR_EN;
  USART1_CR3 &= ~(USART_CR3_DMAT|USART_CR3_DMAR);
}


/* Forward declaration. */
static uint32_t nrf_async_receive_multi_cont();

void
exti3_isr(void)
{
  if (EXTI_PR & EXTI3) {
    /* Clear the pending interrupt event. */
    nrf_irq_clear();

    if (receive_multi_running)
      nrf_idle = nrf_async_receive_multi_cont();
  }
}


/*
  Handler for SPI Rx DMA completed interrupt, serving the nRF24L01+.

  This interrupt is triggered when the transfer to the nRF24L01+ is
  completed, indicating completion of the corresponding command/request.
*/
void
dma2_stream2_isr(void)
{
  if (DMA2_LISR & DMA_LISR_TCIF2)
  {
    /*
      Clear the interrupt request.
      Do this early, so that, if we start another DMA request from within
      the context of this interrupt invocation, and somehow get delayed enough
      that this second DMA can complete before we complete the interrupt
      handler, we will not lose the interrupt of the second DMA request.
    */
    DMA2_LIFCR = DMA_LISR_TCIF2;
    if (nrf_cmd_running)
    {
      ssi_cmd_transfer_done();
      nrf_cmd_running = 0;
      if (receive_multi_running)
        nrf_idle = nrf_async_receive_multi_cont();
    }
  }
}


static void
ssi_cmd_blocking(uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len)
{
  uint32_t i;

  /*
    Note that nRF SPI uses most-significant-bit first, while USART works
    with least-significant-bit first. So we need to bit-swap all the
    bytes sent and received.
  */
  for (i = 0; i < len; ++i)
    nrf_send_buffer[i] = bitswap_byte(sendbuf[i]);

  nrf_cmd_running = 1;
  ssi_cmd_start(len);
  while (nrf_cmd_running)
    ;

  for (i = 0; i < len; ++i)
    recvbuf[i] = bitswap_byte(nrf_recv_buffer[i]);
}


static void
nrf_read_reg_n_blocking(uint8_t reg, uint8_t *out, uint32_t len)
{
  uint8_t sendbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_R_REGISTER | reg;
  memset(&sendbuf[1], 0, len);
  ssi_cmd_blocking(out, sendbuf, len+1);
}


__attribute__((unused))
static uint8_t
nrf_read_reg_blocking(uint8_t reg, uint8_t *status_ptr)
{
  uint8_t recvbuf[2];
  nrf_read_reg_n_blocking(reg, recvbuf, 1);
  if (status_ptr)
    *status_ptr = recvbuf[0];
  return recvbuf[1];
}


static void
nrf_write_reg_n_blocking(uint8_t reg, const uint8_t *data, uint32_t len)
{
  uint8_t sendbuf[6], recvbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd_blocking(recvbuf, sendbuf, len+1);
}


static void
nrf_write_reg_blocking(uint8_t reg, uint8_t val)
{
  nrf_write_reg_n_blocking(reg, &val, 1);
}


static void
nrf_flush_tx()
{
  uint8_t cmd = nRF_FLUSH_TX;
  uint8_t status;
  ssi_cmd_blocking(&status, &cmd, 1);
}


static void
nrf_flush_rx()
{
  uint8_t cmd = nRF_FLUSH_RX;
  uint8_t status;
  ssi_cmd_blocking(&status, &cmd, 1);
}


static void
nrf_init_config(uint32_t channel, uint32_t power)
{
  static const uint8_t addr[3] = { 0x7e, 0xc8, 0x33 };

  nrf_write_reg_blocking(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP);
  /* Disable auto-ack, saving 9 bits/packet. Else 0x3f. */
  nrf_write_reg_blocking(nRF_EN_AA, 0);
  /* Enable only pipe 0. */
  nrf_write_reg_blocking(nRF_EN_RXADDR, nRF_ERX_P0);
  /* 3 byte adresses. */
  nrf_write_reg_blocking(nRF_SETUP_AW, nRF_AW_3BYTES);
  /* Disable auto retransmit. */
  nrf_write_reg_blocking(nRF_SETUP_RETR, 0);
  nrf_write_reg_blocking(nRF_RF_CH, channel);
  /* Use 2Mbps, and set transmit power. */
  nrf_write_reg_blocking(nRF_RF_SETUP, nRF_RF_DR_LOW | power);
  nrf_write_reg_n_blocking(nRF_RX_ADDR_P0, addr, 3);
  nrf_write_reg_n_blocking(nRF_TX_ADDR, addr, 3);
  /* Set payload size for pipe 0. */
  nrf_write_reg_blocking(nRF_RX_PW_P0, 32);
  /* Disable pipe 1-5. */
  nrf_write_reg_blocking(nRF_RX_PW_P1, 0);
  /* Disable dynamic payload length. */
  nrf_write_reg_blocking(nRF_DYNDP, 0);
  /* Allow disabling acks. */
  nrf_write_reg_blocking(nRF_FEATURE, nRF_EN_DYN_ACK);

  /* Clear out all FIFOs. */
  nrf_flush_tx();
  nrf_flush_rx();
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg_blocking(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT);
}


/* Asynchronous SPI transfer to nRF24L01+ using DMA. */

static void
nrf_write_reg_n_start(uint8_t reg, const uint8_t *data, uint32_t len)
{
  uint32_t i;

  if (len > 7)
    len = 7;
  nrf_send_buffer[0] = bitswap_byte(nRF_W_REGISTER | reg);
  for (i = 0; i < len; ++i)
    nrf_send_buffer[i+1] = bitswap_byte(data[i]);
  nrf_cmd_running = 1;
  ssi_cmd_start(len+1);
}


static void
nrf_write_reg_start(uint8_t reg, uint8_t val)
{
  nrf_write_reg_n_start(reg, &val, 1);
}


static void
nrf_read_reg_n_start(uint8_t reg, uint32_t len)
{
  if (len > 7)
    len = 7;
  nrf_send_buffer[0] = bitswap_byte(nRF_R_REGISTER | reg);
  memset(&nrf_send_buffer[1], 0, len);
  nrf_cmd_running = 1;
  ssi_cmd_start(len+1);
}


static void
nrf_read_reg_start(uint8_t reg)
{
  nrf_read_reg_n_start(reg, 1);
}


static void
nrf_rcv_start(uint32_t len)
{
  nrf_send_buffer[0] = bitswap_byte(nRF_R_RX_PAYLOAD);
  memset(&nrf_send_buffer[1], 0, len);
  nrf_cmd_running = 1;
  ssi_cmd_start(len+1);
}


/*
  Continously receive packets on nRF24L01+ in receiver mode.

  As packets are received, they are passed to a callback function. This
  callback is invoked from interrupt context, so it should ideally be fairly
  quick and has to be aware of the general interrupt caveats.
*/
static struct nrf_async_receive_multi {
  void (*consumepacket)(uint8_t *, void *);
  void *cb_data;
  uint8_t state;
} nrf_rcv_state;

enum nrf_async_receive_multi_states {
  ST_NRF_ARM_A1,
  ST_NRF_ARM_A2,
  ST_NRF_ARM_A3,
  ST_NRF_ARM_A4
};

static void
nrf_async_receive_multi_start(void (*consumepacket)(uint8_t *, void *),
                              void *cb_data)
{
  nrf_rcv_state.consumepacket = consumepacket;
  nrf_rcv_state.cb_data = cb_data;
  nrf_rcv_state.state = ST_NRF_ARM_A1;

  /* Assert CE to enter receive mode. */
  ce_high();

  nrf_idle = 1;
  receive_multi_running = 1;

  /*
    Enable interrupt when the IRQ line goes low, which happens when data is
    ready in the Rx FIFO (RX_DR).
  */
  nrf_irq_enable();
}


/*
  Called to continue a multi-packet receive session.
  This should be called when an event occurs, either in the form of
  an SPI DMA completion interrupt or in the form of a GPIO interrupt on the
  nRF24L01+ IRQ pin.

  The two interrupts should be configured to have the same priority, so that
  one of them does not attempt to pre-empt the other; that would lead to
  nasty races.

  Returns 1 if the nRF24L01+ is now idle (no packets pending in Rx FIFO), 0
  if activity is still on-going.
*/
static uint32_t
nrf_async_receive_multi_cont()
{
  uint32_t i;
  uint8_t packet[32];
  struct nrf_async_receive_multi *const a= &nrf_rcv_state;

resched:
  switch (a->state)
  {
  case ST_NRF_ARM_A1:
    nrf_irq_disable();
    /* Clear the RX_DS interrupt. */
    nrf_write_reg_start(nRF_STATUS, nRF_RX_DR);
    a->state = ST_NRF_ARM_A2;
    return 0;

  case ST_NRF_ARM_A2:
    /* Read FIFO status to check if there is any data ready. */
    nrf_read_reg_start(nRF_FIFO_STATUS);
    a->state = ST_NRF_ARM_A3;
    return 0;

  case ST_NRF_ARM_A3:
    if (bitswap_byte(nrf_recv_buffer[1]) & nRF_RX_EMPTY)
    {
      /*
        No more packets in the Rx fifo. Enable the IRQ interrupt and wait for
        more packets to arrive.
      */
      a->state = ST_NRF_ARM_A1;
      /*
        The nRF IRQ is level-triggered, but STM32 only has edge-triggered GPIO
        interrupts. We handle this by first clearing any pending interrupt,
        then checking manually the GPIO pin.
      */
      nrf_irq_clear();
      nrf_irq_enable();
      if (!nrf_irq_pinstatus())
        goto resched;
      return 1;
    }

    /* The Rx FIFO is non-empty, so read a packet. */
    nrf_rcv_start(32);
    a->state = ST_NRF_ARM_A4;
    return 0;

  case ST_NRF_ARM_A4:
    /* Deliver the received packet to the callback. */
    for (i = 0; i < 32; ++i)
      packet[i] = bitswap_byte(nrf_recv_buffer[i+1]);
    (*(a->consumepacket))(packet, a->cb_data);
    /* Now go check if there are more packets available. */
    a->state = ST_NRF_ARM_A2;
    goto resched;

  default:
    /* This shouldn't really happen ... */
    return 1;
  }
}


static void
enqueue_key_event(uint32_t event)
{
  uint32_t tail = key_event_tail;
  uint32_t new_tail = (tail+1) % MAX_KEY_EVENTS;
  if (new_tail != key_event_head)
  {
    key_events[tail] = event;
    key_event_tail = new_tail;
  }
}


static uint32_t
dequeue_key_event(void)
{
  uint32_t head = key_event_head;
  uint32_t event;

  if (head == key_event_tail)
    return KEY_NOEVENT;
  event = key_events[head];
  key_event_head = (head + 1) % MAX_KEY_EVENTS;
  return event;
}


uint32_t
get_key_event(void)
{
  return dequeue_key_event();
}


static void
nrf_receive_cb(uint8_t *packet, void *dummy __attribute__((unused)))
{
  uint8_t cmd, subcmd;

  cmd = packet[0];
  subcmd = packet[1];
  if (cmd == POV_CMD_DEBUG)
  {
    if (subcmd == POV_SUBCMD_RESET_TO_BOOTLOADER)
    {
      /* Reset to bootloader. */
      asm_dsb();
      scb_reset_system();
      asm_dsb();
      for (;;)
        ;
      /* NotReached */
    }
  }
  else if (cmd == POV_CMD_CONFIG)
  {
    if (subcmd == POV_SUBCMD_KEYPRESSES)
    {
      uint32_t old_state = key_state[0] |
        ((uint32_t)key_state[1] << 8) | ((uint32_t)key_state[2] << 16);
      uint32_t new_state = packet[2] |
        ((uint32_t)packet[3] << 8) | ((uint32_t)packet[4] << 16);
      uint32_t i;
      uint32_t diff = old_state ^ new_state;

      memcpy((uint8_t *)key_state, &packet[2], sizeof(key_state));
      for (i = 0; i < 24; ++i)
      {
        if (diff & (1<<i))
          enqueue_key_event(i | ((new_state & (1<<i)) ? KEY_EVENT_DOWN : 0));
      }
    }
  }
}


void
setup_nrf24l01p(void)
{
  setup_nrf_spi();
  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  delay(MCU_HZ/3/10);
  nrf_init_config(81, nRF_RF_PWR_0DBM);
  /* Start receiving packets! */
  nrf_async_receive_multi_start(nrf_receive_cb, NULL);
}


float
joy_r_angle_mag(float *magnitude)
{
  uint32_t hor_raw = key_state[3];
  uint32_t vert_raw = key_state[4];
  float hor = ((float)hor_raw-127.5f) * (1.0f/128.0f);
  float vert = ((float)vert_raw-127.5f) * (1.0f/128.0f);
  float angle;

  if (magnitude)
    *magnitude = sqrtf(hor*hor + vert*vert);
  angle = atan2f(-vert, hor);
  if (angle < 0.0f)
    angle += 2.0f*F_PI;
  return angle;
}


float
joy_l_vert(void)
{
  float val = (127.5f - (float)key_state[6]) * (1.0f/128.0f);
  return val;
}
