#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <string.h>
#include "uhreader.h"

static int debug=0;

#ifdef UART_DTR_RTS_WIEGAND
static int uartdtr(int fd, int stat);
static int uartrts(int fd, int stat);
#endif

enum COMMANDS {
  ///////////////////////////////////////////////////////////////////
  ////////////// EPC C1 G2（ISO18000-6C）COMMAND ////////////////////
  ///////////////////////////////////////////////////////////////////
  // The function is used to inventory tags in the effective field and get their EPC values.
  CMD_INVENTORY         =0x01,
  // The function is used to read part or all of a Tag’s Password, EPC, TID, or User memory.
  // To the word as a unit, start to read data from the designated address.
  CMD_READ_DATA         =0x02,
  // The function is used to write several words in a Tag’s Reserved, EPC, TID, or User memory.
  CMD_WRITE_DATA        =0x03,
  // The function is used to write EPC value in a Tag’s EPC memory. Random write one
  // tag in the effective field.
  CMD_WRITE_EPC         =0x04,
  // The function is used to kill tag. After the tag killed, it never process command.
  CMD_KILL_TAG          =0x05,
  // The function is used to set Password area as readable and writeable from any state,
  // readable and writeable from the secured state, permanently readable and writeable,
  // never readable and writeable. It used to set EPC, TID or User as writeable from any
  // state, writeable from the secured state, permanently writeable, never writeable.
  CMD_LOCK              =0x06,
  // The function is used to erase multiple words in a Tag’s Password, EPC, TID, or User memory.
  CMD_BLOCK_ERASE       =0x07,
  // The function is used to set designated tag read protection. After the tag protected,
  // it never process command. Even if inventory tag, reader can not get the EPC number. 
  // The read protection can be removed by executing Reset ReadProtect. Only NXP's UCODE EPC G2X tags valid. 
  CMD_READ_PROTECT      =0x08,
  // The function is used to random set one tag read protection in the effective field. The tag must be having the same access password. Only NXP's UCODE EPC G2X tags valid. 
  CMD_READ_PROTECT_NOEPC=0x09,
  // The function is used to remove only one tag read protection in the effective field. The tag must be having the same access password. Only NXP's UCODE EPC G2X tags valid. 
  CMD_RESET_READ_PROTECT=0x0A,
  // The function is used to check only one tag in the effective field, whether the tag is protected. It can not check the tag whether the tag support protection setting. Only NXP's UCODE EPC G2X tags valid.
  CMD_CHECK_READ_PROTECT=0x0B,
  //The function is used to set or reset the EAS status bit of designated tag. Only NXP's UCODE EPC G2X tags valid.
  CMD_EAS_ALARM         =0x0C,
  // The function is used to check EAS status bit of any tag in the effective field. Only NXP's UCODE EPC G2X tags valid.
  CMD_CHECK_EAS_ALARM   =0x0D,
  // The function is used to permanently lock the designated data in designated tag’s user memory. The locked data can be read only, but not written and not erased. Only NXP's UCODE EPC G2X tags valid.
  CMD_BLOCK_LOCK        =0x0E,
  // The function is used to inventory one tag in the effective field and get their EPC values.
  CMD_INVENTORY_SINGLE  =0x0F,
  // The function is used to write multiple words in a Tag’s Reserved, EPC, TID, or User memory.
  CMD_BLOCK_WRITE       =0x10,

  ///////////////////////////////////////////////////
  /////////////   18000-6B COMMAND //////////////////
  ///////////////////////////////////////////////////
  // The function is used to Inventory only one tag in the effective field and get their ID values. If more than one tag in the effective field at the same time, reader may be get nothing. 
  CMD_INV_SIGNAL_6B     =0x50,
  // The function is used to according to the given conditions Inventory tags in the effective field and get their ID values.
  CMD_INV_MULTI_6B      =0x51,
  // The function is used to start to read several bytes from the designated address. 
  CMD_READ_DATA_6B      =0x52,
  // The function is used to start to write several bytes from the designated address. 
  CMD_WRITE_DATA_6B     =0x53,
  // The function is used to check whether the designated byte is locked. 
  CMD_CHECK_LOCK_6B     =0x54,
  // The function is used to lock the designated byte.
  CMD_LOCK_6B           =0x55,

  ///////////////////////////////////////////////////////////////////
  ////////////// READER DEFINED COMMAND /////////////////////////////
  ///////////////////////////////////////////////////////////////////
  // This function is used to get reader-related information such as
  // reader address (Adr), firmware version, supported protocol type, Inventory ScanTime, power and frequency.
  CMD_GET_READER_INFO   =0x21,
  // Sets the current region. The function is used to set the reader working of the lower limit and the upper limit of frequency.
  CMD_SET_REGION        =0x22,
  // This function is used to set a new address of the reader. The address value will store in reader’s inner nonvolatile memory. Default address value is 0x00. The value range is 0x00~0xFE. The address 0xFF is reserved as the broadcasting address. When user tries to write a 0xFF to Adr, the reader will set the value to 0x00 automatically. 
  CMD_SET_ADDRESS       =0x24,
  // This function is used to set a new value to Inventory ScanTime of an appointed reader. The range is 3~255 corresponding to 3*100ms~255*100ms Inventory ScanTime. The default value of Inventory ScanTime is 10*100ms. 
  CMD_SET_SCANTIME      =0x25,
  // The function is used to change the serial port baud rate.
  CMD_BAUD_RATE         =0x28,
  // The function is used to set the power of reader.
  CMD_SET_POWER         =0x2F,
  // Acousto-optic Control
  CMD_ACOUSTO_OPTIC_CTL =0x33,
  // BIZZER (undocumented)
  //  05 00 35 80 64 21 -- 05 00 35 00 6c a5    -- set close
  //  05 00 35 81 ed 30 -- 05 00 35 00 6c a5    -- set open
  //
  //  05 00 35 00 6c a5 -- 06 00 35 00 00 03 b4 -- get close
  //  05 00 35 00 6c a5 -- 05 00 35 00 6c a5    -- get open
  CMD_BIZZER_CTL        =0x35, // where manual?
};

static void hexdump(char *msg, const void *datav, int size) {
  unsigned const char *data=datav;
  unsigned const char *x=(unsigned char*)data;
  int i=0;

  if (msg!=NULL) printf("    %i -- %s\n", size, msg);

  printf("   ");
  while (size--) {
    if ((i!=0)&&(i%16) == 0) {
      printf("   \n\r");
    }
    if (
        (*x>' ') &&
        (*x<128)
      )
    i++;
    printf(" %2.2x", *x++);
  }
  printf("\n\r");
}


static int SendDataToPort(uhrdev_t *dev, uint8_t *data, int len) {
  int wlen;
  if (debug) hexdump("Send data", data, len);

  //TODO purgecomm
  wlen=write(dev->fd, data, len);
  if (wlen!=len) return UHRERR_WRITE;
  return 0;
}


int UhrOpenCom(uhrdev_t *uhr, char *fname, uint32_t baud) {
  struct termios options;

  memset(uhr, 0, sizeof(uhr[0]));

  uhr->fd = open(fname, O_RDWR |O_NONBLOCK);
  if (uhr->fd==-1) return -1;

  tcgetattr(uhr->fd, &options);

  cfmakeraw(&options);

  options.c_cc[VINTR]    = 0;
  options.c_cc[VQUIT]    = 0;
  options.c_cc[VERASE]   = 0;
  options.c_cc[VKILL]    = 0;
  options.c_cc[VEOF]     = 0;
  options.c_cc[VSWTC]    = 0;
  options.c_cc[VSTART]   = 0;
  options.c_cc[VSTOP]    = 0;
  options.c_cc[VSUSP]    = 0;
  options.c_cc[VEOL]     = 0;
  options.c_cc[VREPRINT] = 0;
  options.c_cc[VDISCARD] = 0;
  options.c_cc[VWERASE]  = 0;
  options.c_cc[VLNEXT]   = 0;
  options.c_cc[VEOL2]    = 0;

  ////
  options.c_cc[VMIN]     = 0;
  options.c_cc[VTIME]    = 0;

  //
  options.c_cflag &= ~HUPCL; //DTR IS OFF
  options.c_cflag &= ~CLOCAL;
  options.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ECHOCTL | ECHOKE | ICANON | ISIG | IEXTEN);
  options.c_oflag &= ~ONLCR;
  options.c_iflag |= INPCK;//NEED?


  switch(baud) {
    case 57600:
      cfsetispeed(&options, B57600);
      cfsetospeed(&options, B57600);
      break;
    case 19200:
      cfsetispeed(&options, B19200);
      cfsetospeed(&options, B19200);
      break;
    case 115200:
      cfsetispeed(&options, B115200);
      cfsetospeed(&options, B115200);
      break;
    default:
      printf("Unknown band: %i\n", baud);
  }
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  tcsetattr(uhr->fd, TCSANOW, &options);

  uhr->opened=1;
  uhr->addr=0;

#ifdef UART_DTR_RTS_WIEGAND
  uartdtr(uhr->fd, 0); // DTR --
  uartrts(uhr->fd, 0); // DTR --
#endif

  //FIXME NOPARITY
  //      ONESTOPBIT
  //      BYTESIZE = 8
  return 0;
}

#define  POLYNOMIAL   0x8408 
#define  PRESET_VALUE 0xffff
void UhrWriteCRC(uint8_t *pData, int len) {
  int i,j;
  unsigned int current_crc_value = PRESET_VALUE;
  for (i=0;i<len;i++) {
    current_crc_value=current_crc_value^((unsigned int)pData[i]);
    for (j=0; j<8; j++) {
      if (current_crc_value&0x0001)
        current_crc_value=(current_crc_value>>1)^POLYNOMIAL;
      else  current_crc_value=(current_crc_value>>1);
    }
  }
  pData[i++] = (unsigned char)(current_crc_value&0x00ff);
  pData[i] = (unsigned char)((current_crc_value>>8)&0x00ff);
}

static int CheckCRC(uint8_t *pData, int len) {
  UhrWriteCRC(pData,len);
  if ((pData[len + 1] == 0) && (pData[len] == 0))
    return 0;
  else  return 0x31;
}

int UhrClose(uhrdev_t *uhr) {
  if (uhr->fd>=0)
    close(uhr->fd);
  uhr->fd=-1;
  return 0;
}

static int WriteLen(void *begin, void *end) {
  uhrmsg_t *msg=begin;
  msg->len = (uintptr_t)end - (uintptr_t)begin+1;
  return msg->len+1;//+1 CRC -1 for first byte
}
static void WriteCRC(void *begin) {
  uhrmsg_t *msg=begin;
  UhrWriteCRC(msg->raw, msg->len-1);
}

static void *AddInt8(void *vptr, uint8_t data) {
  uint8_t *p=vptr;
  p[0]=data;
  return &p[1];
}

static void *AddInt32(void *vptr, uint32_t data) {
  uint32_t *p=vptr;
  p[0]=htobe32(data);
  return &p[1];
}

// Add data to msg request
// * vptr -- buffer,
// * wlen -- add count (by words)
// * data -- data
// \return -- dataptr
static void *AddData(void *vptr, uint8_t len, char *data) {
  uint8_t *p=vptr;
  p[0]=len;
  len*=2;
  if (len>0) memcpy(&p[1], data, len);
  return &p[1+len];
}

// Write raw data in reverse order
static void *AddRaw(void *vptr, uint8_t len, char *data) {
  uint8_t *p=vptr;
  len*=2;
  memcpy(&p[0], data, len);
  return &p[len];
}

typedef struct _uhrresp_ {
  uhrmsg_t msg;        // 0x00 -- 0x02
  uint8_t  status;
  uint16_t crc;
} uhrmsgresp_t;

// Send command and read answer from device
//
static int CmdToReader(uhrdev_t *dev, uint8_t *data, int size, uint8_t *out, int *len) {
  int     status=0,rlen,rover;
  uint8_t idx=0;
  uint8_t cmd,recv[256];
  int     timeout=1800; // about
  uhrmsgresp_t *resp;

  cmd=data[2];//save old cmd
  if (out==NULL) out=data;
  resp=(uhrmsgresp_t*)out;
  (void)resp;//TODO make the code more readable

  status = SendDataToPort  (dev, data, size);
  if (status!=0) return status;

  memset(recv, 0, 256);
  if (len==NULL) rover=256;
  else           rover=*len;

  //FIXME first byte is len of message, rover must be use it
  while (1) {
    rlen=read(dev->fd, &recv[idx], rover);
    if (rlen>0) idx+=rlen;
    rover-=rlen;
    if (CheckCRC(recv, idx) ==0) break;
    if (rover<=0) {
      status=UHRERR_OVFLOW;
      goto ensure;
    }
    usleep(10);
    if (timeout--<=0) {
      if (debug) printf("ERROR timeout or crc: %i\n", idx);
      if (idx==0) return UHRERR_TIMEOUT;
      else        return UHRERR_CRC;
    }
  }
  if (debug) hexdump("Recv:", recv, idx);

  if (recv[2] != cmd) return UHRERR_UANS;

ensure:
  memcpy(out, recv, idx);
  if (len!=NULL) len[0]=idx;
  if (recv[3] != 0 )  return recv[3];

  return status;
}
// LOW LEVEL API

int UhrGetReaderInformation(uhrdev_t *dev, uhrmsginfo_t *info) {
  info->msg.len  = sizeof(uhrmsg_t)+1;
  info->msg.addr = dev->addr; //WTF addr?? FIXME
  info->msg.cmd  = CMD_GET_READER_INFO;
  UhrWriteCRC(info->msg.raw, info->msg.len-1);

  return CmdToReader(dev, info->msg.raw, info->msg.len + 1, NULL, NULL);
}

int UhrBeep(uhrdev_t *dev, int active, int silent, int times) {
  int           len;
  uint8_t       tbuff[64];
  uint8_t      *vmsg=tbuff;

  vmsg=AddInt8    (vmsg, 0            );  //len
  vmsg=AddInt8    (vmsg, dev->addr    );  //address
  vmsg=AddInt8    (vmsg, CMD_ACOUSTO_OPTIC_CTL);  //cmd
  vmsg=AddInt8    (vmsg, active       );  //active
  vmsg=AddInt8    (vmsg, silent       );  //silent
  vmsg=AddInt8    (vmsg, times        );  //times

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  return CmdToReader(dev, tbuff, len, NULL, NULL);
}

int UhrLock(uhrdev_t *dev, uhrtag_t *tag, int memory, int mode, int pass) {
  int           len;
  uint8_t       tbuff[64];
  uint8_t      *vmsg=tbuff;

  vmsg=AddInt8    (vmsg, 0            );  //len
  vmsg=AddInt8    (vmsg, dev->addr    );  //address
  vmsg=AddInt8    (vmsg, CMD_LOCK     );  //cmd
  if (tag==NULL)
       vmsg=AddData  (vmsg, 0, NULL);
  else vmsg=AddData  (vmsg, tag->wlen, tag->data);
  vmsg=AddInt8    (vmsg, memory);         //mem type
  vmsg=AddInt8    (vmsg, mode);           //mode lock
  vmsg=AddInt32   (vmsg, pass);           //password

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  return CmdToReader(dev, tbuff, len, NULL, NULL);
}

typedef struct _uhrinv_ {
  uhrmsg_t msg;        // 0x00 -- 0x02
  uint8_t status;
  uint8_t count;
  uint8_t mtag[254];// [ [len + EPC], [len+EPC]... ]
  uint16_t crc;
} uhrmsginv_t;

int UhrInventory(uhrdev_t *dev, uhrtag_t *tags, uint16_t *count) {
  int rsize,status,len;
  uint8_t       tbuff[256];
  uint8_t      *vmsg=tbuff;
  uhrmsginv_t  *msg=(void*)tbuff;

  vmsg=AddInt8    (vmsg, sizeof(uhrmsg_t)+1 );  //len
  vmsg=AddInt8    (vmsg, dev->addr          );  //address
  vmsg=AddInt8    (vmsg, CMD_INVENTORY      );  //cmd

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  rsize=sizeof(tbuff);
  status = CmdToReader(dev, tbuff, len, tbuff, &rsize);
  //For Inventory status code '1' is success, not '0'
  if (status!=1) return status;

  {
    int i;
    uhrtag_t *t;
    t=(void*)msg->mtag;
    for (i=0; i < msg->count; i++) {
      if (i>=*count) break;
      tags[i].wlen=t->wlen/2;
      memcpy(tags[i].data,t->data,t->wlen);

      t=(void*)(((uint8_t*)t)+sizeof(t->wlen)+t->wlen);//FIXME checkit
    }
  }
  *count=msg->count;
  return 0;
}


int UhrReadCard(uhrdev_t *dev, uhrtag_t *tag, int mtype, int off, int count, uint32_t pass, int mask, int mlen, void *buff, int *plen) {
  int          status,rsize;
  uint8_t      tbuff[256];
  void        *vmsg=tbuff;
  int          len;

  vmsg=AddInt8    (vmsg, 0            );  //len
  vmsg=AddInt8    (vmsg, dev->addr    );  //address
  vmsg=AddInt8    (vmsg, CMD_READ_DATA);  //cmd
  if (tag==NULL)
       vmsg=AddData  (vmsg, 0, NULL);
  else vmsg=AddData  (vmsg, tag->wlen, tag->data);

  vmsg=AddInt8    (vmsg, mtype);          //memory type TID/EPC/USER/Password
  vmsg=AddInt8    (vmsg, off);            //start (by word)
  vmsg=AddInt8    (vmsg, count);          //read count (by word)
  vmsg=AddInt32   (vmsg, pass);           //password
  if (mask>0) {
    vmsg=AddInt8    (vmsg, mask);
    vmsg=AddInt8    (vmsg, mlen);
  }

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  rsize=256;
  status = CmdToReader(dev, tbuff, len, tbuff, &rsize);
  if (status!=0) {
    if (status == 0xFC)
      status=0x70+tbuff[4];
    return status;
  }

  len=rsize-4-2;//4 -- msg req, 2 -- crc
  if (plen!=NULL) *plen=len;
  memcpy(buff, &tbuff[4], len);
  return 0;
}

// * dev   -- device by UhrOpen
// * tag   -- tag by UhrInventory
// * count -- cound words to write
// * mtype -- UHRMEM_?
// * off   -- word offset read from
// * buff  -- words array to write
// * pass  -- password (0 by default)
//   #TODO mask  -- EPC compare mask FIXME
//   #TODO mlen  -- mask len
//

// FIXME add FULL variant with EPC MASK
int UhrWriteCard(uhrdev_t *dev, int wcount, uhrtag_t *tag, int mtype, int off, void *buff, uint32_t pass, int mask, int mlen) {
  int           len;
  uint8_t       tbuff[256];
  void         *vmsg=tbuff;

  vmsg=AddInt8    (vmsg, 0            );  //len
  vmsg=AddInt8    (vmsg, dev->addr    );  //address
  vmsg=AddInt8    (vmsg, CMD_WRITE_DATA); //cmd
  vmsg=AddInt8    (vmsg, wcount);         //read count (by word)
  if (tag==NULL)
       vmsg=AddData   (vmsg, 0, NULL);    //EPC
  else vmsg=AddData   (vmsg, tag->wlen, tag->data);
  vmsg=AddInt8    (vmsg, mtype);          //memory type TID/EPC/USER/Password
  vmsg=AddInt8    (vmsg, off);            //start (by word)

  vmsg=AddRaw     (vmsg, wcount, buff);

  vmsg=AddInt32   (vmsg, pass);           //password
  if (mask>0) {
    vmsg=AddInt8    (vmsg, mask);
    vmsg=AddInt8    (vmsg, mlen);
  }

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  return CmdToReader(dev, tbuff, len, NULL, NULL);
}

int UhrWriteEPC(uhrdev_t *dev, char *data, int wlen, uint32_t pass) {
  int           len;
  uint8_t       tbuff[256];
  void         *vmsg=tbuff;

  vmsg=AddInt8    (vmsg, 0            );  //len
  vmsg=AddInt8    (vmsg, dev->addr    );  //address
  vmsg=AddInt8    (vmsg, CMD_WRITE_EPC); //cmd
  vmsg=AddInt8    (vmsg, wlen         );  //len
  vmsg=AddInt32   (vmsg, pass         );  //pass
  vmsg=AddRaw     (vmsg, wlen, data   );  //EPC id

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  return CmdToReader(dev, tbuff, len, NULL, NULL);
}

/// HI LEVEL API///

// uhrtaginfo_t
// Read 32 bytes from TID (TID EPC/TMD/TMDID/TMN)
int UhrReadCardInfo(uhrdev_t *dev, uint32_t pass, uhrtaginfo_t *tinfo) {
  int          status,rsize,len;
  uint8_t      tbuff[256];
  void        *vmsg=tbuff;

  vmsg=AddInt8    (vmsg, 0            );  //len
  vmsg=AddInt8    (vmsg, dev->addr    );  //address
  vmsg=AddInt8    (vmsg, CMD_READ_DATA);  //cmd
  vmsg=AddData    (vmsg, 0, NULL);
  vmsg=AddInt8    (vmsg, UHRMEM_TID);     //mem type
  vmsg=AddInt8    (vmsg, 0);              //start
  vmsg=AddInt8    (vmsg, 2);              //world count 
  vmsg=AddInt32   (vmsg, pass);           //password

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  status = CmdToReader(dev, tbuff, len, tbuff, &rsize);
  if (status!=0) return status;

  memcpy(tinfo,tbuff+4,sizeof(uhrtaginfo_t));
  return 0;
}

int UhrReadCardTID(uhrdev_t *dev, uhrtag_t *tag, uint32_t pass, uhrtaginfo_t *taginfo, uint64_t *tid) {
  int          status,rsize,len;
  uint8_t      tbuff[256];
  void        *vmsg=tbuff;

  vmsg=AddInt8    (vmsg, 0            );  //len
  vmsg=AddInt8    (vmsg, dev->addr    );  //address
  vmsg=AddInt8    (vmsg, CMD_READ_DATA);  //cmd
  if (tag==NULL)
       vmsg=AddData  (vmsg, 0, NULL);
  else vmsg=AddData  (vmsg, tag->wlen, tag->data);
  vmsg=AddInt8    (vmsg, UHRMEM_TID);     //mem type
  vmsg=AddInt8    (vmsg, 0);              //start
  vmsg=AddInt8    (vmsg, 6);              //world count 
  vmsg=AddInt32   (vmsg, pass);           //password

  len = WriteLen(tbuff, vmsg);
  WriteCRC(tbuff);

  rsize=256;
  status = CmdToReader(dev, tbuff, len, tbuff, &rsize);
  if (debug) printf("TID rsize: %i, status %x\n", rsize, status);
  if (status!=0) return status;

  if (rsize!=(12+4+2)) return 1;

  memcpy(taginfo, &tbuff[4], 4);
  //memcpy(tid, &tbuff[6], 8);

  //*taginfo=be16toh(*((uint16_t*)&tbuff[4]));
  *tid    =be64toh(*((uint64_t*)&tbuff[8]));

  //memcpy(data,tbuff+4,16);
  //hexdump("TBUFF", data, 26);
  return 0;
}

int UhrReadCardPass(uhrdev_t *dev, uint32_t pass, uint32_t *acc, uint32_t *kill) {
  int          status,len;
  uint8_t      tbuff[256];

  status = UhrReadCard(dev, NULL, UHRMEM_PASS, 0, 4, pass, -1, -1, tbuff, &len);
  if (status!=0) return 1;

  hexdump("Pass: ", tbuff, 8);
  memcpy(acc , &tbuff[0], 4);
  memcpy(kill, &tbuff[4], 4);

  return 0;
}

int UhrReadCardEPC(uhrdev_t *dev, uint32_t pass, uhrtaginfo_t *taginfo, uint64_t *tid) {
  int          status,len;
  uint8_t      tbuff[256];

  status = UhrReadCard(dev, NULL, UHRMEM_EPC, 1, 7, pass, -1, -1, tbuff, &len);
  if (status!=0) return 1;

  len=tbuff[0]>>3;
  memcpy(taginfo, &tbuff[2], 4);
  *tid    =be64toh(*(uint64_t*)&tbuff[6]);
  return 0;
}


////////////////////////////////////////////////////////////
/// Unducumented function wiegand send (Send by DTR RTX)

#ifdef UART_DTR_RTS_WIEGAND
static int uartdtr(int fd, int stat) {
  int res,flags;

  res=ioctl(fd,TIOCMGET,&flags);
  if (res<0) return 1;
  if (stat) flags |= TIOCM_DTR;
  else      flags &=~TIOCM_DTR;
  res=ioctl(fd,TIOCMSET,&flags);

  return 0;
}

static int uartrts(int fd, int stat) {
  int res,flags;

  res=ioctl(fd,TIOCMGET,&flags);
  if (res<0) return 1;
  if (stat) flags |= TIOCM_RTS;
  else      flags &=~TIOCM_RTS;
  res=ioctl(fd,TIOCMSET,&flags);

  return 0;
}

#define WGPULSETIME 2
#define WGPAUSETIME 20
static int sendone(int fd) {
  uartdtr(fd, 1); // DTR --
  usleep(WGPULSETIME);
  uartdtr(fd, 0); // DTR --
  if (debug) printf("1");
  return 0;
}

static int sendzero(int fd) {
  uartrts(fd, 1); // RTS --
  usleep(WGPULSETIME);
  uartrts(fd, 0); // RTS --
  if (debug) printf("0");
  return 0;
}

static int senduint32_t(int fd, uint32_t cmdle) {
  int i;
  int tmp;
  int odd=1,even=0;
  //uint32_t cmd=htobe32(cmdle);
  uint32_t cmd=cmdle;

  for (tmp=0,i=0; i<16; i++)
    if ((cmd>>i)&1) tmp++;
  if ((tmp%2)==0) odd=0;

  for (tmp=0,i=16; i<32; i++)
    if ((cmd>>i)&1) tmp++;
  if ((tmp%2)==0) even=1;

  if (even)  sendzero(fd);
  else       sendone(fd);
  usleep(WGPAUSETIME);

  if (debug) printf(" ");
  for (i=0; i<32; i++) {
    int b = (cmd>>(31-i))&1;
    if (b) sendone(fd);
    else   sendzero(fd);
    usleep(WGPAUSETIME);
    if ((i%8)==7) if (debug) printf(" ");
  }

  if (odd) sendzero(fd);
  else     sendone(fd);
  usleep(WGPAUSETIME);
  return 0;
}
int UhrWgSend(uhrdev_t *uhr, uint32_t wgcmd) {
  senduint32_t(uhr->fd, wgcmd);
  usleep(10000);
  return 0;
}
#endif
