#include "ledtorus.h"
#include "nrf24l01p.h"


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


uint8_t volatile key_state;
#define MAX_KEY_EVENTS 16

static volatile uint32_t key_events[MAX_KEY_EVENTS];
static volatile uint32_t key_event_head = 0, key_event_tail = 0;


static void
nrf_irq_enable(void)
{
  EXTI->IMR |= EXTI_Line3;
}


static void
nrf_irq_disable(void)
{
  EXTI->IMR &= ~EXTI_Line3;
}


static void
nrf_irq_clear(void)
{
  EXTI->PR = EXTI_Line3;
}


static uint32_t
nrf_irq_pinstatus(void)
{
  return GPIOC->IDR & GPIO_Pin_3;
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
  union {
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    DMA_InitTypeDef DMA_InitStructure;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
  } u;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  USART_Cmd(USART1, DISABLE);

  /*
    Clock on PA8.
    Polarity is idle low, active high.
    Phase is sample on rising, setup on falling edge.
  */
  u.GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  u.GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  u.GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  u.GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  u.GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOA, &u.GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_USART1);

  /* MOSI and MISO on PB6/PB7. */
  u.GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  u.GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  u.GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  u.GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  u.GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOB, &u.GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

  /*
    CS on PC1, CE on PC2.
    CS is high initially (active low).
    CE is low initially (active high).
  */
  GPIO_SetBits(GPIOC, GPIO_Pin_1);
  GPIO_ResetBits(GPIOC, GPIO_Pin_2);
  u.GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;
  u.GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  u.GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  u.GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  u.GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &u.GPIO_InitStructure);

  u.USART_InitStructure.USART_BaudRate = 5250000;
  u.USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  u.USART_InitStructure.USART_StopBits = USART_StopBits_1;
  u.USART_InitStructure.USART_Parity = USART_Parity_No;
  u.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  u.USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
  USART_Init(USART1, &u.USART_InitStructure);
  u.USART_ClockInitStruct.USART_Clock = USART_Clock_Enable;
  u.USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;
  u.USART_ClockInitStruct.USART_CPHA = USART_CPHA_1Edge;
  u.USART_ClockInitStruct.USART_LastBit = USART_LastBit_Enable;
  USART_ClockInit(USART1, &u.USART_ClockInitStruct);

  USART_Cmd(USART1, ENABLE);

  /* Setup DMA. USART1 on DMA2 channel 4, streams 2 (Rx) and 7 (Tx). */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  DMA_DeInit(DMA2_Stream2);
  DMA_DeInit(DMA2_Stream7);

  u.DMA_InitStructure.DMA_BufferSize = 1;
  u.DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  u.DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  u.DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  u.DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  u.DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  u.DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  u.DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  u.DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  u.DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  u.DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
  /* Configure USART1 TX DMA */
  u.DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART1->DR));
  u.DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  u.DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  u.DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA2_Stream7, &u.DMA_InitStructure);
  /* Configure USART1 RX DMA */
  u.DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  u.DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  u.DMA_InitStructure.DMA_Memory0BaseAddr = 0;
  DMA_Init(DMA2_Stream2, &u.DMA_InitStructure);

  /* Configure a USART1 DMA Rx transfer complete interrupt. */
  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, DISABLE);
  u.NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream2_IRQn;
  u.NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 10;
  u.NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  u.NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&u.NVIC_InitStruct);
  DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, ENABLE);

  /* IRQ on PC3. */
  u.GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  u.GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  u.GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  u.GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  u.GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &u.GPIO_InitStructure);

  /* Take an interrupt on falling edge (IRQ is active low). */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource3);
  u.EXTI_InitStruct.EXTI_Line = EXTI_Line3;
  u.EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  u.EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  u.EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_Init(&u.EXTI_InitStruct);
  /*
    We only want to configure the interrupt here, not yet enable it. But the
    ST libraries seem to not have a way to configure the interrupt without
    also enabling it. So let's just configure it with the library routine
    and immediately disable it.
  */
  nrf_irq_disable();

  /* Clear any pending interrupt before enabling in NVIC. */
  nrf_irq_clear();
  u.NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
  u.NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 10;
  u.NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  u.NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&u.NVIC_InitStruct);
}


static inline void
csn_low(void)
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_1);
}


static inline void
csn_high(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_1);
}


static inline void
ce_low(void)
{
  GPIO_ResetBits(GPIOC, GPIO_Pin_2);
}


static inline void
ce_high(void)
{
  GPIO_SetBits(GPIOC, GPIO_Pin_2);
}


static inline uint8_t
bitswap_byte(uint8_t in)
{
  return (uint8_t)(__RBIT((uint32_t)in) >> 24);
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

  DMA2_Stream2->M0AR = (uint32_t)(&nrf_recv_buffer[0]);
  DMA2_Stream2->NDTR = len;
  DMA2_Stream7->M0AR = (uint32_t)(&nrf_send_buffer[0]);
  DMA2_Stream7->NDTR = len;
  /* Clear DMA transfer complete flags. */
  DMA2->LIFCR = DMA_FLAG_TCIF2 & 0x0F7D0F7D;
  DMA2->HIFCR = DMA_FLAG_TCIF7 & 0x0F7D0F7D;
  /* Clear the  USART TC (transfer complete) flag. */
  USART1->SR &= ~USART_FLAG_TC;
  /* Enable the Rx and Tx DMA channels. */
  DMA2_Stream2->CR |= DMA_SxCR_EN;
  DMA2_Stream7->CR |= DMA_SxCR_EN;
  /* Enable the USART1 to generate Rx/Tx DMA requests. */
  USART1->CR3 |= (USART_DMAReq_Tx|USART_DMAReq_Rx);
}


static void
ssi_cmd_transfer_done()
{
  /* Take CSN high to complete transfer. */
  csn_high();

  /* Disable DMA requests and channels. */
  DMA2_Stream2->CR &= ~DMA_SxCR_EN;
  DMA2_Stream7->CR &= ~DMA_SxCR_EN;
  USART1->CR3 &= ~(USART_DMAReq_Tx|USART_DMAReq_Rx);
}


/* Forward declaration. */
static uint32_t nrf_async_receive_multi_cont();

void
EXTI3_IRQHandler(void)
{
  if (EXTI->PR & EXTI_Line3) {
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
DMA2_Stream2_IRQHandler(void)
{
  if (DMA2->LISR & 0x00200000)
  {
    /*
      Clear the interrupt request.
      Do this early, so that, if we start another DMA request from within
      the context of this interrupt invocation, and somehow get delayed enough
      that this second DMA can complete before we complete the interrupt
      handler, we will not lose the interrupt of the second DMA request.
    */
    DMA2->LIFCR = 0x00200000;
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
      NVIC_SystemReset();
      /* NotReached */
    }
  }
  else if (cmd == POV_CMD_CONFIG)
  {
    uint8_t old_state = key_state;
    uint8_t new_state = packet[2];

    if (subcmd == POV_SUBCMD_KEYPRESSES)
    {
      uint32_t i;
      uint32_t diff = old_state ^ new_state;

      key_state = new_state;
      for (i = 0; i < 8; ++i)
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
