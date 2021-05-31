#ifndef __UHREADER_H__
#define __UHREADER_H__
#include <inttypes.h>

// Switch on wiegand output (DTR, RTS)
#define UART_DTR_RTS_WIEGAND 1

#pragma pack(push,1)
typedef struct {
  union {
    struct {
      uint8_t  len;
      uint8_t  addr;
      uint8_t  cmd;
      //uint8_t  crc;//send crc
    };
    uint8_t raw[3];
  };
} uhrmsg_t;

typedef struct _uhrreader_ {
  int fd;
  int addr;
  int opened;
} uhrdev_t;

typedef struct _ltag_ {
  uint8_t wlen;     // len in words 0..15
  char    data[32]; // 30 -- max len TAG
} uhrtag_t;

typedef struct _tid_ {
  union {
    struct {
      uint8_t  class;
      uint8_t  manuf_hi;
      uint8_t  model:4;
      uint8_t  manuf_lo:4;
      union {
        struct {
          uint8_t  vermin:4;
          uint8_t  vermaj:4;
        };
        uint8_t model_lo;
      };
    };
    uint32_t raw;
  };
} uhrtaginfo_t;

typedef struct _uhrinfo_ {
  uhrmsg_t msg;        // 0x00 -- 0x03
  uint16_t version;    // 0x04
  uint8_t  model;      // 0x06
  uint8_t  supro;      // 0x07    support protocol
  uint8_t  dmaxfre;    // 0x08
  uint8_t  dminfre;    // 0x09
  uint8_t  power;      // 0x10
  uint8_t  scantime;   // 0x11
  uint8_t  crc;        // 0x12
  uint8_t  padding[3]; // 0x13-0x16
} uhrmsginfo_t;
#pragma pack(pop)

enum {
  UHRLT_KLL =0x00,
  UHRLT_ACC =0x01,
  UHRLT_EPC =0x02,
  UHRLT_TID =0x03,
  UHRLT_USR =0x04,
};

enum {
  UHRLM_W_ANY  =0x00,
  UHRLM_W_P    =0x01,
  UHRLM_W_SEC  =0x02,
  UHRLM_W_NO   =0x03,
};

enum {
  UHRMEM_PASS=0,
  UHRMEM_EPC =1,
  UHRMEM_TID =2,
  UHRMEM_USER=3
};

enum URHERRORS {
  // 0x00-0x19 ; 0xf9-0xff -- TAG or READER errors
  UHRERR_GOOD     =0x00, // GOOD
  UHRERR_NOTAG    =0xFB,

  //////////////////////// System errors
  UHRERR_NOTAGS   =0x20,
  UHRERR_MANYTAGS =0x21, // many tags
  UHRERR_GOODFIX  =0x22, // Tag is fixed, all good
  UHRERR_CANTLOCK =0x23, // can't lock password
  UHRERR_WRITE    =0x30, // Error write to reader
  UHRERR_CRC      =0x31,
  UHRERR_TIMEOUT  =0x32,
  UHRERR_OVFLOW   =0x38,
  UHRERR_MREAD    =0x55,// misread 
  UHRERR_UANS     =0xEE,
  ///// TAG ERRR 0x70 + tag error code
  UHRERR_TERR_OTHER    =0x70, // 0 -- other error
  UHRERR_TERR_OVERRUN  =0x73, // 3 -- overrrun memory
  UHRERR_TERR_LOCKED   =0x74, // 4 -- memory locked
  UHRERR_TERR_POWER    =0x7b, // b -- insufficient power
  UHRERR_TERR_ANY      =0x7f, // tag unsupported TERR code

  ///
  UHRERR_POORCOMM =0xFA,// Are some tags in the effective field, but Poor Communication between reader and tag.
  UHRERR_INVALID  =0xFF,
};

/// uhr   -- object
/// fname -- device filename, example: /dev/ttyUSB0
/// baud  -- baud rate
///
/// \retrun 0 -- success
///        !0 -- error
int UhrOpenCom(uhrdev_t *uhr, char *fname, uint32_t baud);


/// Close object (created by UhrOpenCom)
/// dev      -- device (created by UhrOpenCom)
///
/// \retrun 0 -- success
///        !0 -- error
int UhrClose(uhrdev_t *dev);


/// Read all tags (EPC)
/// dev      -- device (created by UhrOpenCom)
/// tags     -- tags array
/// count    -- size of tags array
///
/// \retrun 0 -- success
///        !0 -- error
int UhrInventory(uhrdev_t *dev, uhrtag_t *tags, uint16_t *count);

/// Read from tag
/// * dev   -- device by UhrOpen
/// * tag   -- tag by UhrInventory
/// * mtype -- UHRMEM_?
/// * off   -- word offset read from
/// * count -- count words read
/// * pass  -- password (0 by default)
/// * mask  -- EPC compare mask FIXME
/// * mlen  -- mask len
/// * buff  -- read data to buff MUST be >= (count*2 + 6)
/// * len   -- bytes real read
///
/// \retrun 0 -- success
///        !0 -- error
int UhrReadCard(uhrdev_t *dev, uhrtag_t *tag, int mtype, int off, int count, uint32_t pass, int mask, int mlen, void *buff, int *plen);


/// Write to tag
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
///
/// \retrun 0 -- success
///        !0 -- error
int UhrWriteCard(uhrdev_t *dev, int wcount, uhrtag_t *tag, int mtype, int off, void *buff, uint32_t pass, int mask, int mlen);


/// Write EPC to any tag
/// * dev   -- device by UhrOpen
/// * data  -- tag data
/// * wlen  -- tag len
/// * pass  -- password (0 by default)
///
/// \retrun 0 -- success
///        !0 -- error
int UhrWriteEPC(uhrdev_t *dev, char *data, int wlen, uint32_t pass);


/// Read TID (hi level API)
/// * dev     -- device (created by UhrOpenCom)
/// * tag     -- EPC tag (maybe NULL for any)
/// * pass    -- password (0 by default)
/// * taginfo -- 
/// * tid     -- 
///
/// \return   0 -- success
///          !0 -- error
///
int UhrReadCardTID(uhrdev_t *dev, uhrtag_t *tag, uint32_t pass, uhrtaginfo_t *taginfo, uint64_t *tid);

/// Get Reader Info
/// dev      -- device (created by UhrOpenCom)
/// info     -- save info into
///
/// \return   0 -- success
///          !0 -- error
int UhrGetReaderInformation(uhrdev_t *dev, uhrmsginfo_t *info);


/// Beep
/// dev      -- device (created by UhrOpenCom)
/// active   -- active time
/// silent   -- silent time
/// times    -- repeat time
///
/// \return   0 -- success
///          !0 -- error
int UhrBeep(uhrdev_t *dev, int active, int silent, int times);


/// Lock for Read/Write access
///
/// * dev    -- device by UhrOpen
/// * tag    -- tag by UhrInventory
/// * memory -- memory type UHRMEM_
/// * mode   -- lock mode UHRLM_W_
/// * pass   -- password
int UhrLock(uhrdev_t *dev, uhrtag_t *tag, int memory, int mode, int pass);


////////// HI LEVEL API ////////////

/// * dev     -- device by UhrOpen
/// * pass    -- password (0 by default)
/// * taginfo --
/// * tid     --
///
/// \return   0 -- success
///          !0 -- error
int UhrReadCardEPC(uhrdev_t *dev, uint32_t pass, uhrtaginfo_t *taginfo, uint64_t *tid);
int UhrReadCardPass(uhrdev_t *dev, uint32_t pass, uint32_t *acc, uint32_t *kill);


/// DTR RTS wiegand output
int UhrWgSend(uhrdev_t *uhr, uint32_t wgcmd);

#endif //__UHREADER_H__
