
/* This header file describes the CSR Slave for the tx_ehip_dma component */

#ifndef __TX_EHIP_DMA_CSR_REGS_H__
#define __TX_EHIP_DMA_CSR_REGS_H__



/******************************************************************************/
/* Memory Map Summary                                                         */
/******************************************************************************/

/*
  Register  | Access  |   Register Contents      | Description
  Address   |         |      (64-bits)           | 
------------|---------|--------------------------|-----------------------------
        0x0 |       R |         {reserved[62:0], |     Read the busy status of
            |         |               busy[0:0]} |               the component
            |         |                          |  0 - the component is ready
            |         |                          |       to accept a new start
            |         |                          |    1 - the component cannot
            |         |                          |          accept a new start
------------|---------|--------------------------|-----------------------------
        0x8 |       W |         {reserved[62:0], |  Write 1 to signal start to
            |         |              start[0:0]} |               the component
------------|---------|--------------------------|-----------------------------
       0x10 |     R/W |         {reserved[62:0], |      0 - Disable interrupt,
            |         |   interrupt_enable[0:0]} |        1 - Enable interrupt
------------|---------|--------------------------|-----------------------------
       0x18 |  R/Wclr |         {reserved[61:0], | Signals component completion
            |         |               done[0:0], |       done is read-only and
            |         |   interrupt_status[0:0]} | interrupt_status is write 1
            |         |                          |                    to clear
------------|---------|--------------------------|-----------------------------
       0x20 |       R |         {reserved[31:0], |                 Return data
            |         |        returndata[31:0]} |                            

NOTE: Writes to reserved bits will be ignored and reads from reserved
      bits will return undefined values.
*/

struct ehip_dma_tx_csr {
	u32 reserved1;
	u32 busy;
	u32 reserved2;
	u32 start;
     u32 reserved3;
     u32 interrupt_enable;
     u32 reserved4;
     u32 status;
     u32 reserved5;
     u32 tx_completions;
};

/******************************************************************************/
/* Register Address Macros                                                    */
/******************************************************************************/

/* Byte Addresses */
#define TX_EHIP_DMA_CSR_BUSY_REG (0x0)
#define TX_EHIP_DMA_CSR_START_REG (0x8)
#define TX_EHIP_DMA_CSR_INTERRUPT_ENABLE_REG (0x10)
#define TX_EHIP_DMA_CSR_INTERRUPT_STATUS_REG (0x18)
#define TX_EHIP_DMA_CSR_RETURNDATA_REG (0x20)

/* Argument Sizes (bytes) */
#define TX_EHIP_DMA_CSR_RETURNDATA_SIZE (4)

/* Argument Masks */
#define TX_EHIP_DMA_CSR_RETURNDATA_MASK (0xffffffff)

/* Status/Control Masks */
#define TX_EHIP_DMA_CSR_BUSY_MASK   (1<<0)
#define TX_EHIP_DMA_CSR_BUSY_OFFSET (0)

#define TX_EHIP_DMA_CSR_START_MASK   (1<<0)
#define TX_EHIP_DMA_CSR_START_OFFSET (0)

#define TX_EHIP_DMA_CSR_INTERRUPT_ENABLE_MASK   (1<<0)
#define TX_EHIP_DMA_CSR_INTERRUPT_ENABLE_OFFSET (0)

#define TX_EHIP_DMA_CSR_INTERRUPT_STATUS_MASK   (1<<0)
#define TX_EHIP_DMA_CSR_INTERRUPT_STATUS_OFFSET (0)
#define TX_EHIP_DMA_CSR_DONE_MASK   (1<<1)
#define TX_EHIP_DMA_CSR_DONE_OFFSET (1)


#endif /* __TX_EHIP_DMA_CSR_REGS_H__ */



