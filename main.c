#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>


//WG
#include <sys/ioctl.h>
#include <openssl/sha.h>

#include <uhreader.h>


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

unsigned char rBuff[5000];

  
static int debug=1;


int UhrPrintInfo(uhrmsginfo_t *info) {
  printf("COM addr     : %x, cmd: %x\n", info->msg.addr, info->msg.cmd);
  printf("Version      : %x\n", info->version);
  printf("Support Proto: %x\n", info->supro);
  printf("Model        : %x\n", info->model);
  printf("dMaxFre      : %x\n", info->dmaxfre);
  printf("dMinFre      : %x\n", info->dminfre);
  printf("power        : %x\n", info->power);
  printf("scantime     : %x\n", info->scantime);
  printf("CRC          : %x\n", info->crc);
  
  return 0;
}

int UhrPrintTag(uhrtag_t *tag) {
  // I don't found any info about decoding TAG EPC...

  {
    int i;
    printf("TAG (%i): ", tag->wlen*2);
    for (i=0;i<tag->wlen*2;i++) printf(" %2.2x", (unsigned char)tag->data[i]);
    printf("\n");
  }
  return 0;
}

int UhrPrintTagInfo(uhrtaginfo_t *taginfo) {
  uint32_t  mf=0;
  if (taginfo->class!=0xe2) {
     printf("Unknown CLASS: %x\n",taginfo->class);
     return 1;
  }
  printf("Class EPC Global ACI. ");
  mf=((uint32_t)taginfo->manuf_hi<<4)|(taginfo->manuf_lo);

  switch(mf) {
    case 0x03: {
      printf("Alien. ");
      switch(taginfo->model) {
        case 0x4: printf("Higgs 4 v.%i.%i ",taginfo->vermaj,taginfo->vermin);
        break;
      } 
      break;
      }
    case 0x806: {
      uint16_t model=((uint16_t)taginfo->model<<8)|(taginfo->model_lo);
      printf("NXP ");
      switch(model) {
        case 0x894: printf("UCODE 8"); break;
        default: printf("Unknown model: 0x%x", model); break;
      }
      break;
      }
    case 0x801: {
      uint16_t model=((uint16_t)taginfo->model<<8)|(taginfo->model_lo);
      printf("Impinj ");
      switch(model) {
        case 0x105: printf("Monza 4QT"); break;
        case 0x160: printf("Monza R6"); break;
        default: printf("Unknown model"); break;
      }
      break;
      }
    default:
      printf("Unknown manufactor 0x%x. ",mf);
      break;
  }
  printf("\n");
  return 0;
}

static int TestGetReaderInfo(uhrdev_t *uhr) {
  int          status;
  uhrmsginfo_t devinfo;

  status = UhrGetReaderInformation(uhr, &devinfo);
  if (status != 0) {
    printf("Error get info\n");
    return -1;
  }
  UhrPrintInfo(&devinfo);
  return 0;
}

static int TestInventory(uhrdev_t *uhr, uhrtag_t *tag) {
  int      i;
  int      status;
  uint16_t count=0;
  uhrtag_t tags[256];
  //
  status = UhrInventory(uhr, tags, &count);
  if (status!=0) return -1;
  printf("Tags found: %i\n", count);
  for (i=0; i<count; i++) {
    UhrPrintTag(&tags[i]);
  }
  if (count>0) {
    *tag=tags[0];
  }
  return 0;
}

static int TestRead(uhrdev_t *uhr, uhrtag_t *tag, char *buff) {
  int  len;
  int  status;

  status = UhrReadCard   (uhr, tag, UHRMEM_USER, 0, 4, 0x0000, -1, -1, buff, &len);
  if (status != 0) {
    printf("Read error: %x\n", status);
    return 1;
  }
  hexdump("read: ", buff, len);


  (void)uhr;

  return 0;
}

static int TestWrite(uhrdev_t *uhr, uhrtag_t *tag, char *buff) {
  int  status;

  printf("*** TEST WRITE CARD\n");
  hexdump("writed: ", buff, 2*2);
  status = UhrWriteCard(uhr, 2, tag, UHRMEM_USER, 0, buff, 0x0000, -1, -1);
  if (status != 0) {
    printf("Write error: %x\n", status);
    return 1;
  }


  (void)uhr;

  return 0;
}

int UhrGetPassword(uhrdev_t *uhr, uhrtag_t *tag, uint32_t *kill, uint32_t *acc) {
  int  len=8,status;
  uint32_t buff[2];

  status = UhrReadCard   (uhr, tag, UHRMEM_PASS, 0, 8/2, 0x0000, -1, -1, buff, &len);
  if (status != 0) return status;
  if (debug) printf("Get pass, len: %i\n", len>>3);
  // FIXME maybe mix up
  kill[0]=be32toh(buff[0]);
  acc[0] =be32toh(buff[1]);
  return 0;
}

int UhrSetPassword(uhrdev_t *uhr, uhrtag_t *tag, uint32_t kill, uint32_t acc) {
  int  status;
  uint32_t buff[2];
  buff[0]=htobe32(kill);
  buff[1]=htobe32(acc);

  status = UhrWriteCard(uhr, 8/2, tag, UHRMEM_PASS, 0, buff, 0x0000, -1, -1);
  if (status != 0) {
    printf("ERROR SetPassoword\n");
    return 1;
  }
  return 0;
}
int UhrBaseLock(uhrdev_t *uhr, uhrtag_t *tag, uint32_t pass) {
  int status;
  status = UhrLock(uhr, tag, UHRLT_KLL , UHRLM_W_SEC, pass);
  if (status!=0) {
    printf("Error lock KILL PASS: %x\n", status);
    return status;
  }
  status = UhrLock(uhr, tag, UHRLT_ACC , UHRLM_W_SEC, pass);
  if (status!=0) {
    printf("Error lock ACC: %x\n", status);
    return status;
  }
  status = UhrLock(uhr, tag, UHRLT_EPC , UHRLM_W_SEC, pass);
  if (status!=0) {
    printf("Error lock EPC: %x\n", status);
    return status;
  }
  return 0;
}

int UhrBaseUnLock(uhrdev_t *uhr, uhrtag_t *tag, uint32_t pass) {
  int status;
  status = UhrLock(uhr, tag, UHRLT_KLL , UHRLM_W_ANY, pass);
  if (status!=0) {
    printf("Error unlock KILL PASS: %x\n", status);
    return status;
  }
  status = UhrLock(uhr, tag, UHRLT_ACC , UHRLM_W_ANY, pass);
  if (status!=0) {
    printf("Error unlock ACC: %x\n", status);
    return status;
  }
  status = UhrLock(uhr, tag, UHRLT_EPC , UHRLM_W_ANY, pass);
  if (status!=0) {
    printf("Error unlock EPC: %x\n", status);
    return status;
  }
  //status = UhrLock(uhr, tag, UHRLT_TID , UHRLM_W_SEC, pass);
  //if (status!=0) {
  //  printf("Error lock TID: %x\n", status);
  //  return status;
  //}
  return 0;
}


int TestUnLock(uhrdev_t *uhr, uhrtag_t *tag, uint32_t pass) {
  int status;
  status = UhrLock(uhr, tag, UHRLT_KLL , UHRLM_W_ANY, pass);
  if (status!=0) {
    printf("Error lock KILL PASS: %x\n", status);
    return status;
  }
  status = UhrLock(uhr, tag, UHRLT_ACC , UHRLM_W_ANY, pass);
  if (status!=0) {
    printf("Error lock ACC: %x\n", status);
    return status;
  }
  status = UhrLock(uhr, tag, UHRLT_EPC , UHRLM_W_ANY, pass);
  if (status!=0) {
    printf("Error lock EPC: %x\n", status);
    return status;
  }
  return 0;
}

int main_t(void) {
  int       i,status;
  uhrtag_t  tag;
  uhrdev_t  uhr_obj;
  uhrdev_t *uhr=&uhr_obj;
  char      buff[256];

  //status = UhrOpenCom(uhr, "/dev/ttyUSB0", 115200);
  //status = UhrOpenCom(uhr, "/dev/ttyUSB1", 57600);
  status = UhrOpenCom(uhr, "/dev/ttyUSB0", 57600);
  //status = UhrOpenCom(uhr, "/dev/ttyUSB0", 115200);
  //status = UhrOpenCom(uhr, "/dev/pts/23", 57600);
  if (status!=0) {
    printf("Can't open UHR reader\n");
    return 1;
  }
  if (0) TestGetReaderInfo(uhr);
  status = TestInventory(uhr, &tag);
  if (status!=0) {
    printf("Error found TAG\n");
    goto exit_code;
  }
  if (0) {
    status = TestRead(uhr, &tag, buff);
    if (status!=0) {
      printf("Read error\n");
      goto exit_code;
    }
    for (i=0; i<8; i++) buff[i]^=0x55;
    TestWrite(uhr, &tag, buff);
  }
  if (1) {
    int status;
    uint32_t kill,acc;
    status = UhrSetPassword(uhr, &tag, 0x11111111, 0x44332211);
    if (status!=0) {
      printf("Error set pass\n");
      goto exit_code;
    }
    status = UhrGetPassword(uhr, &tag, &kill, &acc);
    if (status !=0 ) {
      printf("error get pass\n");
      goto exit_code;
    }
    printf("Pass kill: %x, access: %x\n", kill, acc);
  }
  if (0) {
    status = UhrBaseLock(uhr, &tag, 0x44332211);
    if (status!=0) {
      goto exit_code;
    }
    printf("Locket success\n");
  }
  if (0) {
    status = TestUnLock(uhr, &tag, 0x44332211);
    if (status!=0) {
      goto exit_code;
    }
    printf("Locket success\n");
  }

exit_code:
  printf("UHR close...\n");
  UhrClose(uhr);
  return 0;
}

char *progname;
static int usage() {
  printf("Usage: %s <opts>\n",progname);
  printf("   opts:\n");
  printf("      -h          - help message\n");
  printf("      -d<device>  - device name (default: /dev/ttyUSB0)\n");
  printf("      -v          - verbose mode\n");
  printf("      -I          - get info about tag\n");
  printf("      -u          - unlock tag\n");
  printf("      -f          - get tag, check tag, fix tag if need\n");
  printf("      -p[pass hi] - set master password first\n");
  printf("      -p[pass lo] - set master password second\n");
  printf("      -w<WGCMD>   - send WG command (DTR -- zero, RTS -- one)\n");

  return 1;
}

enum {
  CMD_CARDINFO, // Read any tag and return TID
  CMD_BEEP,
  CMD_UNLOCK,
  CMD_CHECKFIX,
  CMD_WG,
};

int UhrCalcPass(uint64_t TID, uint64_t MasterPass[2], uint32_t *kill, uint32_t *acc) {
  SHA256_CTX    ctx;
  uint8_t       buffer[64];
  uint8_t       result[SHA_DIGEST_LENGTH];
  uint64_t      tidbe=htobe64(TID);
  uint64_t      p1=htobe64(MasterPass[0]);
  uint64_t      p2=htobe64(MasterPass[1]);

  memcpy(&buffer[0] , &tidbe  , 8);
  memcpy(&buffer[8] , &p1     , 8);
  memcpy(&buffer[16], &p2     , 8);

  if (debug) printf("Password calculate\n");
  if (debug) hexdump("PASS FROM", buffer, 24);
  SHA256_Init(&ctx);
  SHA256_Update(&ctx, buffer, 24);
  SHA256_Final(result, &ctx);

  memcpy(kill, &result[0], 4);
  memcpy(acc , &result[4], 4);
  if (debug) printf("   KILL: %x, ACC: %x\n", *kill, *acc);

  return 0;
}

//static int CmpEPCandTID(uhrtag_t *tag, uhrtaginfo_t *epc);


// MUST BE 12 bytes
static void TagTID2tag(uhrtag_t *tag, uhrtaginfo_t *info, uint64_t *TID) {
  uint64_t htid=be64toh(*TID);
  tag->wlen=6;
  memcpy(&tag->data[0], info  , sizeof(info[0]));
  memcpy(&tag->data[4], &htid , 8);
}

static int BeepCmd(uhrdev_t *uhr, int cmd) {
  (void)uhr;
  switch(cmd) {
    case UHRERR_GOOD: {
        printf("***********************\n");
        printf("*        GOOD         *\n");
        printf("***********************\n");
        UhrBeep(uhr, 1, 1, 1);
        break;}
    case UHRERR_GOODFIX: {
        printf("***********************\n");
        printf("*        GOOD FIX     *\n");
        printf("***********************\n");
        UhrBeep(uhr, 1, 1, 2);
        break; }
    case UHRERR_POORCOMM: {
        printf("***********************\n");
        printf("* POOR COMMUNTICATION *\n");
        printf("***********************\n");
        break; }
    case UHRERR_NOTAGS: {
        printf("***********************\n");
        printf("*       NO TAGS       *\n");
        printf("***********************\n");
        break; }
    case UHRERR_MANYTAGS: {
        printf("***********************\n");
        printf("*     TO MANY TAGS    *\n");
        printf("***********************\n");
        break; }
    case UHRERR_CANTLOCK: {
        printf("***********************\n");
        printf("*   CAN'T LOCK TAG    *\n");
        printf("***********************\n");
        break; }
    default:
        printf("***********************\n");
        printf("* UNKNOWN MESS: %2.2x    *\n",cmd);
        printf("***********************\n");
        break;
  }
  return 0;
}

static int TagChangeEPC(uhrdev_t *uhr, uhrtag_t *tag, uhrtag_t *tid, int single) {
  uint16_t len;
  uint8_t  buff[32]; 
  int status;
  
  if (debug) printf("Read LEN EPC\n");
  status = UhrReadCard(uhr, tag, UHRMEM_EPC, 1, 1, 0x0000, -1, -1, &len, NULL);

  len&=~(0x1f<<3);
  len|=tid->wlen<<3;

  memcpy(&buff[0],&len,2);
  memcpy(&buff[2],tid->data,tid->wlen*2);

  if (debug) printf("EPC write 7 bytes 1 stage mode with tag\n");
  status = UhrWriteCard(uhr, tid->wlen+1, tag, UHRMEM_EPC, 1, &buff,  0x0000, -1, -1);


  if (status==UHRERR_INVALID) {
    if (single==1) {
      printf("EPC change error, try change without tag\n");
      status = UhrWriteEPC(uhr, tid->data, tid->wlen, 0x0000);
      if (status!=0) return UHRERR_POORCOMM;
    } else {
      return UHRERR_MANYTAGS;
    }
  }

  return status;
}


static int TagCheckFixPass(uhrdev_t *uhr, uhrtag_t *tag, uint64_t MasterPass[2], uint64_t TID) {
  int      fixed=0;
  int      status,unlock=0;
  uint32_t pkill,pacc;
  uhrtaginfo_t taginfo;
  uint64_t ETID;

  if (debug) printf("Check LOCK memory password\n");
  status = UhrGetPassword(uhr, tag, &pkill, &pacc);
  if (status==0) unlock=1;
  else {
    if (status!=UHRERR_TERR_LOCKED) return status;
  }
  printf("Status: %i\n", status);

  UhrCalcPass(TID, MasterPass, &pkill, &pacc);

  if (unlock==1) {
    if (debug) printf("Write password\n");
    status = UhrSetPassword(uhr, tag, pkill, pacc);
    if (status!=0) return status;

    if (debug) printf("Lock memory\n");
    status = UhrBaseLock(uhr, tag, pacc);
    if (status!=0) return status;
    fixed=1;

    if (debug) printf("Check locked memory\n");
    status = UhrGetPassword(uhr, tag, &pkill, &pacc);
    if (status==0) return UHRERR_CANTLOCK;
  }
  
  if (debug) printf("Check EPC password\n");
  status = UhrReadCardEPC(uhr, pacc, &taginfo, &ETID);
  if (status!=0) return UHRERR_CANTLOCK;

  if (fixed==1) return UHRERR_GOODFIX;
  return UHRERR_GOOD;
}

static uint32_t xor32(uint64_t a) {
  return a^(a>>32);
}


int CheckAndFix(uhrdev_t *uhr, uint64_t MasterPass[2]) {
  int           status,tnum;
  uint16_t      count=1;
  int           fixed=0;
  uhrtag_t      tidtag;
  uhrtag_t      tags[256];
  uhrtag_t     *tag;
  uint64_t      TID;
  uhrtaginfo_t  taginfo;

  if (debug) printf("Inventory tags\n");
  status = UhrInventory(uhr, tags, &count);
  if ((status!=0)||(count<=0)) return UHRERR_NOTAGS;
  tag=&tags[0];
  if (debug) printf("Found: %i tags, work with: \n", count);

  for (tnum=0; tnum<count; tnum++) {
    UhrPrintTag(&tags[0]);

    if (debug) printf("Read TID\n");
    status = UhrReadCardTID(uhr, tag, 0x00000000, &taginfo, &TID);
    TagTID2tag(&tidtag, &taginfo, &TID);

    if (memcmp(&tidtag, &tags[0], 13)==0) {
      fixed=1;
    } else {
      int singlemode=0;
      if (count==1) singlemode=1;
      status = TagChangeEPC(uhr, tag, &tidtag, singlemode);
      if (status==UHRERR_GOOD) fixed=1;
    }
    if (fixed) {
      printf("TAG INFO: %x\n", taginfo.raw&0xfffff);
      if ((taginfo.raw&0xfffff)==0x180e2) { //Monza R6 do not supported passowrds
        uint32_t t32=xor32(TID);
        
        printf("Minoza R6 send to WG without pass %lx (%llx)\n", (long)t32, (long long)TID);
        UhrWgSend(uhr, t32);//32 bites
        return status;
      }
    }
    if (status!=UHRERR_GOOD) return status;
    status = TagCheckFixPass(uhr, tag, MasterPass, TID);
  }
  if ((status==UHRERR_GOOD)&&(fixed==1)) status=UHRERR_GOODFIX;

  if (status==UHRERR_GOOD)
      UhrWgSend(uhr, TID);//32 bites

  return status;
}



int main(int argc, char *argv[]) {
  int opt;
  int status;
  int badpass=0;
  int verb=0;
  int cmd=CMD_CARDINFO;//default CMD
  char devfname[64]="/dev/ttyUSB0";
  int  baud=57600;
  uhrdev_t  uhr_obj;
  uhrdev_t *uhr=&uhr_obj;
  uhrtaginfo_t taginfo;
  uhrtaginfo_t rtaginfo;
  uint64_t TID;
  uint64_t ETID;
  uint32_t pkill,pacc;//kill password and access passowrd
  uint64_t masterpass[2];
  uint32_t wgcmd=0;

  masterpass[0]=0x0000000000000000l;
  masterpass[1]=0x0000000000000000l;

  progname=argv[0];
  printf("UHR reader\n");

  while ((opt=getopt(argc, argv, "fhvd:p:P:buw:")) != -1) {
    switch(opt) {
      case 'h': usage(); return 0;
      case 'd': strcpy(devfname, optarg); break;
      case 'v': verb=1; break;
      case 'p': masterpass[0]=strtoll(optarg, NULL, 16); break;
      case 'P': masterpass[1]=strtoll(optarg, NULL, 16); break;
      case 'b': cmd=CMD_BEEP; break;
      case 'f': cmd=CMD_CHECKFIX; break;
      case 'u': cmd=CMD_UNLOCK; break;
      case 'w': {
                  wgcmd=strtoll(optarg, NULL, 16);
                  cmd=CMD_WG;
                  break;
                }

      default:
                usage();
                return 0;
    }
  }
  if (verb==0) debug=0;

  if (verb) printf("Open device: %s baud: %i\n", devfname, baud);
  status = UhrOpenCom(uhr, devfname, baud);
  if (status!=0) {
    printf("Can't open UHR reader\n");
    return 1;
  }
  switch(cmd) {
    case CMD_CARDINFO:
      //status = UhrReadCardInfo(uhr, 0x00000000, &taginfo);
      if (verb) printf("Read card TID\n");
      status = UhrReadCardTID(uhr, NULL, 0x00000000, &rtaginfo, &TID);
      if (status!=0) {
        printf("Error read card\n");
        return 1;
      }
      if (status==0) {
        UhrPrintTagInfo(&rtaginfo);
        printf("ID: 0x%x, TID: 0x%llx MPASS: 0x%16.16llx 0x%16.16llx\n",be32toh(*(uint32_t*)&rtaginfo),
            (long long)TID,
            (long long) masterpass[0],
            (long long) masterpass[1]);
      }
      status = UhrCalcPass(TID, masterpass, &pkill, &pacc);
      printf("Passwords\n * kill 0x%8.8x\n * acc: 0x%8.8x\n", pkill, pacc);



      printf("TID:%x\n", rtaginfo.raw);
      if ((rtaginfo.raw&0xfffff)==0x180e2) { //Monza R6 do not supported passowrds
        //pkill=pacc=0;

        printf("R6 checking\n");
        status = UhrReadCardEPC(uhr, 0, &taginfo, &ETID);
        if (taginfo.raw!=rtaginfo.raw) {
          printf("WRONG TAG\n");
        }
        if (ETID!=TID) {
          printf("WRONG TID\n");
        }
        if (status ==0 ) {
          printf("EPC correct, success\n");
        } else {
          badpass=1;
          printf("EPC uncorrect, unsuccess\n");
          return 1;
        }
        return 0;

      }

      status = UhrReadCardEPC(uhr, pacc, &taginfo, &ETID);
      if (status !=0 ) {
        badpass=1;
        printf("** Can't read PASS EPC, try without pass...\n");
        status = UhrReadCardEPC(uhr, 0x0000, &taginfo, &ETID);
      }

      if (status ==0 ) {
        printf("EP: 0x%x, TID: 0x%llx",be32toh(*(uint32_t*)&taginfo), (long long)ETID);
        if (badpass) printf(", _BAD PASS_");
        else         printf(", PASS SUCC");
        if (ETID==TID)  printf(", EPC OK");
        else          { printf(", EPC ERR %lx!=%lx",ETID,TID);  }
      } else { printf("Error read EPC %i", status);  }

      if (badpass==0) {
        uint32_t kill,acc;
        status = UhrGetPassword(uhr, NULL, &kill, &acc);
        if (status == 0) {
          printf(" *** UNLOCK PASSOWRD ***");
        } else 
          printf(", PASSWORD LOCKED");
      }
      printf("\n");
      return 0;
    case CMD_BEEP: {
        status = UhrBeep(uhr, 1, 1, 2);
        if (status==0) printf("Beep success\n");
        else printf("Beep error\n");
        break;
      }
    case CMD_UNLOCK: {
        int status;
//        uhrtag_t      tags[256];
        uint32_t pkill,pacc;//kill password and access passowrd
  
 //       status = UhrInventory(uhr, tags, &count);

        status = UhrReadCardTID(uhr, NULL, 0x00000000, &rtaginfo, &TID);
        if (status!=0) {printf("ERROR READ TID\n"); return 1;}
        status = UhrCalcPass(TID, masterpass, &pkill, &pacc);
        printf("PASS: %x\n", pacc);

        status = UhrBaseUnLock(uhr, NULL, pacc);

        return status;
        //return CheckAndFix(uhr);
        }
      break;
    case CMD_CHECKFIX: {
      while (1) {
        status = CheckAndFix(uhr, masterpass);
        BeepCmd(uhr, status);
        sleep(1);
      }
      break;
      }
    case CMD_WG: {
      UhrWgSend(uhr, wgcmd);
      //sleep(10);
      break;
      }

  }
  return 0;
}
