

#ifndef  __STRUCT_DEFINE_H__
#define  __STRUCT_DEFINE_H__

/* define frame types */
#define FETCH_RATIO		0	 //主机为平衡电桥,请求从机回发SlaveRatioofVHMVEART
#define BALANCE_BRIDGE  1	 //主机为平衡电桥,请求主机回发VHM与VEART数据帧
#define	P3N5_BRIDGE	    2	 //主机正极接300k负极接500k
#define	P5N3_BRIDGE	    3	 //主机正极接500k负极接300k

#define FRAME_TYPE_NUM	 4	 

/* define communication status */
#define COMMU_OK 0x0000
#define COMMU_NG 0x0001

/* define whether imblance bridge is enabled */
#define IBRG_ON  0X0001	
#define IBRG_OFF 0x0000

/* indicate if calibrate the ratio of VEART and VHM */
#define YES 0
#define NO  1

/*
 * communication protocol transfermation module type
 */
enum e_rs485_mod
{
    master_mod=0, 
    slave_mod, 
};

/* 
 * system set parameter
 *
 *
 */
struct s_system_set_para
{
    enum e_rs485_mod rs485_mod;	 /* to indicate the status of RS485 type, latched when power on */
    uint8 dip_input; 
    uint8 can_addr;

};

/* 
 * system status
 *
 *
 */
struct s_system_status
{
	uint8 	bridge_type;	 	//	current bridge status, including BALANCE_BRIDGE / P3N5_bridge / P5N3_bridge
//	uint8 	VHM_valid;			//to indicate whether voltage fluctuates due to open/close switches
//	uint8 	VEART_valid;		//to indicate whether voltage fluctuates due to open/close switches
	uint16 	RatioOfVHMVEART;	//本机的VEART与VHM的比例
	uint16 	SlaveRatioOfVHMVEART;//接收到的从机的VEART与VHM的比例
	uint16 	InjectResistance;	 //相窜处阻值

} __attribute__ ((packed));

/*
 * received frame 
 *
 *
 */

struct s_recv_modbus_frame
{
    uint8 addr;
    uint8 function_code;
    uint8 reg_hi;
    uint8 reg_low;
    uint8 data_hi;
    uint8 data_low;

} __attribute__ ((packed));



#endif

