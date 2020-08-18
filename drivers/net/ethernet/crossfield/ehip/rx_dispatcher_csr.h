
/* This header file describes the CSR Slave for the rx_dispatcher component */

#ifndef __RX_DISPATCHER_CSR_REGS_H__
#define __RX_DISPATCHER_CSR_REGS_H__



/******************************************************************************/
/* Memory Map Summary                                                         */
/******************************************************************************/

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
------------|---------|--------------------------|-----------------------------
       0x28 |     R/W |         {reserved[31:0], |             Argument addr32
            |         |            addr32[31:0]} |                            
------------|---------|--------------------------|-----------------------------
       0x30 |     R/W |         {reserved[31:0], |             Argument addr64
            |         |            addr64[31:0]} |                            
------------|---------|--------------------------|-----------------------------
       0x38 |     R/W |         {reserved[31:0], |             Argument length
            |         |            length[31:0]} |                            

NOTE: Writes to reserved bits will be ignored and reads from reserved
      bits will return undefined values.
*/


struct rx_ehip_dma_dispatcher {
     u32 busy;
     u32 reserved1;
	u32 start;
     u32 reserved2;
     u32 interrupt_enable;
     u32 reserved3;
     u32 status;
     u32 reserved4;
     u32 fill_level;
     u32 reserved5;
	u32 addr32;	/* data buffer source address low bits */
     u32 reserved6;
	u32 addr64;	/* data buffer destination address low bits */
     u32 reserved7;
	u32 length;	/* the number of bytes to transfer
				 * per descriptor
				 */
     u32 reserved8;
};

/******************************************************************************/
/* Register Address Macros                                                    */
/******************************************************************************/

/* Byte Addresses */
#define RX_DISPATCHER_CSR_BUSY_REG (0x0)
#define RX_DISPATCHER_CSR_START_REG (0x8)
#define RX_DISPATCHER_CSR_INTERRUPT_ENABLE_REG (0x10)
#define RX_DISPATCHER_CSR_INTERRUPT_STATUS_REG (0x18)
#define RX_DISPATCHER_CSR_RETURNDATA_REG (0x20)
#define RX_DISPATCHER_CSR_ARG_ADDR32_REG (0x28)
#define RX_DISPATCHER_CSR_ARG_ADDR64_REG (0x30)
#define RX_DISPATCHER_CSR_ARG_LENGTH_REG (0x38)

/* Argument Sizes (bytes) */
#define RX_DISPATCHER_CSR_RETURNDATA_SIZE (4)
#define RX_DISPATCHER_CSR_ARG_ADDR32_SIZE (4)
#define RX_DISPATCHER_CSR_ARG_ADDR64_SIZE (4)
#define RX_DISPATCHER_CSR_ARG_LENGTH_SIZE (4)

/* Argument Masks */
#define RX_DISPATCHER_CSR_RETURNDATA_MASK (0xffffffff)
#define RX_DISPATCHER_CSR_ARG_ADDR32_MASK (0xffffffff)
#define RX_DISPATCHER_CSR_ARG_ADDR64_MASK (0xffffffff)
#define RX_DISPATCHER_CSR_ARG_LENGTH_MASK (0xffffffff)

/* Status/Control Masks */
#define RX_DISPATCHER_CSR_BUSY_MASK   (1<<0)
#define RX_DISPATCHER_CSR_BUSY_OFFSET (0)

#define RX_DISPATCHER_CSR_START_MASK   (1<<0)
#define RX_DISPATCHER_CSR_START_OFFSET (0)

#define RX_DISPATCHER_CSR_INTERRUPT_ENABLE_MASK   (1<<0)
#define RX_DISPATCHER_CSR_INTERRUPT_ENABLE_OFFSET (0)

#define RX_DISPATCHER_CSR_INTERRUPT_STATUS_MASK   (1<<0)
#define RX_DISPATCHER_CSR_INTERRUPT_STATUS_OFFSET (0)
#define RX_DISPATCHER_CSR_DONE_MASK   (1<<1)
#define RX_DISPATCHER_CSR_DONE_OFFSET (1)



#endif /* __RX_DISPATCHER_CSR_REGS_H__ */



