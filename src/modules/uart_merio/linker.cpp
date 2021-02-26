#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include "linker.h"


const float Linker::VEL_MAX_RAD_S = 1.996f;

const uint16_t Linker::_CRCtab[256]  = {
	    0, 4129, 8258,12387,16516,20645,24774,28903,33032,37161,41290,45419,49548,53677,57806,61935, 4657,  528,12915, 8786,21173,17044,29431,25302,37689,33560,45947,41818,54205,50076,62463,58334,
	 9314,13379, 1056, 5121,25830,29895,17572,21637,42346,46411,34088,38153,58862,62927,50604,54669,13907, 9842, 5649, 1584,30423,26358,22165,18100,46939,42874,38681,34616,63455,59390,55197,51132,
	18628,22757,26758,30887, 2112, 6241,10242,14371,51660,55789,59790,63919,35144,39273,43274,47403,23285,19156,31415,27286, 6769, 2640,14899,10770,56317,52188,64447,60318,39801,35672,47931,43802,
	27814,31879,19684,23749,11298,15363, 3168, 7233,60846,64911,52716,56781,44330,48395,36200,40265,32407,28342,24277,20212,15891,11826, 7761, 3696,65439,61374,57309,53244,48923,44858,40793,36728,
	37256,33193,45514,41451,53516,49453,61774,57711, 4224,  161,12482, 8419,20484,16421,28742,24679,33721,37784,41979,46042,49981,54044,58239,62302,  689, 4752, 8947,13010,16949,21012,25207,29270,
	46570,42443,38312,34185,62830,58703,54572,50445,13538, 9411, 5280, 1153,29798,25671,21540,17413,42971,47098,34713,38840,59231,63358,50973,55100, 9939,14066, 1681, 5808,26199,30326,17941,22068,
	55628,51565,63758,59695,39368,35305,47498,43435,22596,18533,30726,26663, 6336, 2273,14466,10403,52093,56156,60223,64286,35833,39896,43963,48026,19061,23124,27191,31254, 2801, 6864,10931,14994,
	64814,60687,56684,52557,48554,44427,40424,36297,31782,27655,23652,19525,15522,11395, 7392, 3265,61215,65342,53085,57212,44955,49082,36825,40952,28183,32310,20053,24180,11923,16050, 3793, 7920};

Linker::Linker()
{
/*
	initCRCtab();
    for(int i=0;i<256;i++)
    {
    	if(i%32==0) printf("\n");
    	printf("%5i,", _CRCtab[i]);


    }
*/
}
/*
void Linker::initCRCtab(void)
{
#define CRCkey	0x1021
    int16_t i, j;
    uint16_t crc, c;
    for ( i = 0 ; i < 256 ; i++)
    {
        crc = 0;
        c = i << 8;
        for ( j = 0 ; j < 8 ; j++)
        {
            if ((crc ^ c) & 0x8000)
            {
                crc = (crc << 1) ^ CRCkey;
            }
            else
            {
                crc = crc << 1;
            }
            c = c << 1;
        }
        _CRCtab[i] = crc;
    }
}
*/
uint16_t Linker::CRC16(uint8_t* buffer, uint16_t len, uint16_t crc)
{
    uint16_t tmp;
    uint16_t i;
    for(i = 0 ; i < len ; i++)
    {
        tmp = (crc >> 8) ^ buffer[i];
        crc = (crc << 8) ^ _CRCtab[tmp];
    }
    return crc;
}

bool Linker::connectWith(int fd)
{
	_serialPort = fd;
    return true;
}



bool Linker::getConnectStatus()
{
    return _isConnected;
}

void Linker::addBlocGENPAYLOAD(uint8_t blocId)
{
    _sendBuffer[_sendPtr++]=2; /* set LEN */
    _sendBuffer[_sendPtr++]=GENPAYLOAD; /* CLASS */
    _sendBuffer[_sendPtr++]=blocId; /* ID */
    _lenPos = _sendPtr -3;
    switch(blocId)
    {
    case B_REQCMD:
        _sendBuffer[_sendPtr++] = B_STATUS;
        if(!_isConnected)
        {
            _sendBuffer[_sendPtr++] = B_SN;
            _sendBuffer[_sendPtr++] = B_PN;
            _sendBuffer[_sendPtr++] = B_SOFTMAIN;
            _sendBuffer[_sendPtr++] = B_GIMBALCONFIG;
            _sendBuffer[_sendPtr++] = B_MODELCAMDAY;
            _sendBuffer[_sendPtr++] = B_MODELCAMIR;
        }
        break;
/*
    case B_POSITION:
        _sendBuffer[_lenPos] = 8;
        X32ToInt16(_posRefAzi,0.005729577951300823); //scall_isConnecteding modified to convert degree to rad
        X32ToInt16(_posRefEle,0.005729577951300823);
        X32ToInt16(0.5, 6e-5);
        addLEN(); // sould be 8
        break;
    case B_ICR:
        _sendBuffer[_lenPos] = 3;
        if(_ICRReq)
        {
            _sendBuffer[_sendPtr++] = 0x01;
            _ICRReq = false;
        }
        else
        {
            _sendBuffer[_sendPtr++] = 0x00;
            _ICRReq = true;
        }
        addLEN(); // should be 3
        break;
    case B_NUC: // no payload (LEN=2)
    	addLEN();
        break;
    case B_IRPOLARITY:// no payload (LEN=2)
    	addLEN();
        break;
    case B_ACK:// no payload (LEN=2)
    	addLEN();
        break;
        */
    }
}

uint8_t Linker::addLEN()
{
	uint8_t len = 0;
	if(_lenPos!=0) {
		len = _sendPtr-_lenPos-1;
		_sendBuffer[_lenPos] = len;//LEN
		_lenPos = 0;
	}
	return len;
}

void Linker::receive(uint8_t data)
{
    _isReading = true;
    data &= 0xff;
//    qDebug() << "received : " << QString::asprintf("0x%02X (%u)",data, data);
    if (data == 0xAA)
    {
//        qDebug() << "0xAA received";
        decode(data, 1);
        _x5Adetected = 0;
    }
    else
    {
        if (_x5Adetected == 1)
        {
            _x5Adetected = 0;
            if(data == 0x00)
            {
                decode(0xAA, 0);
            }
            else
            {
                decode(0x5A, 0);
            }
        }
        else
        {
            if(data == 0x5A)
            {
                _x5Adetected = 1;
            }
            else
            {
                decode(data, 0);
            }
        }
    }
    _isReading = false;
}
void Linker::decode(uint8_t data, int16_t status)
{
    if(status == 1)
    {
        _ptr = 0;
        _checkFlag = 0;
        _state = LEN;
    }
    else if(_state == LEN)
    {
        _len = data;
        _buffer[_ptr++] = _len;
        if ((_len == 0) || (_len > sizeof(_buffer)))
        {
            _state = START;
        }
        else
        {
            _state = DATA;
        }
    }
    else if (_state == DATA)
    {
        _buffer[_ptr++] = data;
        if (_ptr == (_len+1))
        {
            _state = CHECK;
        }
    }
    else if (_state == CHECK)
    {
        if (_checkFlag == 0)
        {
            _checksum = data;
            _checkFlag = 1;
        }
        else
        {
            _state = START;
            _checksum += data << 8;
            if (_checksum == CRC16(_buffer, _len + 1, 0x00))
            {
//                qDebug() << "CHECKSUM OK";
                reader(&_buffer[1], _len);
            }
            else
            {
            	_errors_crc++;
//                qDebug() << "CHECKSUM ERROR";
            }
        }
    }
}
void Linker::reader(uint8_t* data, int16_t len)
{
    int16_t ptr = 0;
    uint8_t id;
    id = data[0];
//    qDebug() << "reader ID " << QString::asprintf("0x%02X (%u)",id, id);
//    qDebug() << "len : " << len;
    if (id == 0)
    {
        data++;
        while (ptr < (len-1))
        {
//            qDebug() << "ptr : " << ptr;
//            qDebug() << "len bloc : " << QString::asprintf("%u",data[ptr]) ;
            int16_t add = ptr + data[ptr] + 1;
//            qDebug() << "next : " << add;
            if(add <= len)
            {
                unbloc(&data[ptr]);
            }
            else
            {
//                qDebug() << "bloc trop long";
            }
            ptr = add;
        }
    }
}
void Linker::unbloc(uint8_t* buffer)
{
	Header *header = (Header*)&buffer[0];
    uint8_t *payload = &buffer[3];

    setBlocReceived(header->ID);
    //printf("unbloc class-id 0x%02X- 0x%02X\n",classe, id);
    switch(header->ID)
    {
    case B_SN:
        _isConnected = true;
        readString(&payload[0], _SN);
//        printf("Serial Number: %s\n",str);
        break;
    case B_PN:
        readString(&payload[0], _PN);
//    	printf("Part Number : %s\n",str);
        break;
    case B_SOFTMAIN:
        readString(&payload[0], _SWmain);
//    	printf("Software : %s\n",str);
        break;
    case B_MODELCAMDAY:
        //readString(&payload[0], _cameraDay);
        break;
    case B_MODELCAMIR:
        //readString(&payload[0], _cameraIR);
        break;
    case B_STATUS:
    	if(header->LEN == (sizeof(_status)+sizeof(Header)-1)){
			memcpy(&_status, payload, sizeof(_status));
    	}
        break;
    case B_TEMPERATURE:
    	if(header->LEN == (sizeof(_temperature)+sizeof(Header)-1)){
			memcpy(&_temperature, payload, sizeof(_temperature));
    	}
        break;
    case B_LLHTARGET:
    	if(header->LEN == (sizeof(_target)+sizeof(Header)-1)){
			memcpy(&_target, payload, sizeof(_target));
	    	//printf("traget %i, %i, %im\n", _target.lat_rad, _target.lon_rad, _target.alt_m);
    	}
        break;
    case B_LLHTARGETREF:
    	if(header->LEN == (sizeof(_target_ref)+sizeof(Header)-1)){
			memcpy(&_target_ref, payload, sizeof(_target_ref));
	    	//printf("traget ref %i, %i, %im\n", _target_ref.lat_rad, _target_ref.lon_rad, _target_ref.alt_m);
    	}
        break;
    case B_LLHUAV:
    	if(header->LEN == (sizeof(_llh_uav)+sizeof(Header)-1)){
			memcpy(&_llh_uav, payload, sizeof(_llh_uav));
    	}
        break;
    case B_EULERUAV:
    	if(header->LEN == (sizeof(_euler_uav)+sizeof(Header)-1)){
			memcpy(&_euler_uav, payload, sizeof(_euler_uav));
    	}
        break;
    case B_GIMBALCONFIG:
    	if(header->LEN == (sizeof(_config)+sizeof(Header)-1)){
			memcpy(&_config, payload, sizeof(_config));
    	}
        break;
    default:
    	//printf("Received Unhandled Bloc of ID :%i and size: %i\n", header->ID, header->LEN);
        break;
    }

}

// ========= Writing Blocs must call addLen after each addBloc
void Linker::addBloc(uint8_t blocClass, uint8_t blocId)
{
    switch(blocClass)
    {
    case GENPAYLOAD:
        addBlocGENPAYLOAD(blocId);
        break;
    case LRFPAYLOAD:
        /*addBlockLRFPAYLOAD(blockId);*/
        break;
    default:
        //printf("Classe non implémentée\n");
        break;
    }
}
void Linker::sendProtocol()
{
    int16_t i;
    uint16_t checksum;
    send(0xAA);
    _sendBuffer[0] = _sendPtr - 1; // length of all blocs
    _sendBuffer[1] = 0x00;	// payload id, fixed to 0x00
    for ( i = 0 ; i < _sendPtr ; i++)
    {
        sendByte(_sendBuffer[i]);
    }
    checksum = CRC16(_sendBuffer, _sendPtr, 0x00);
    sendByte(checksum & 0xff);
    sendByte(checksum >> 8);
#ifndef __PX4_NUTTX
    if(_socket !=-1){
    	int len = sendto(_socket, _socketBuffer, _socketBufferSize , 0 , (struct sockaddr *) &_si_remote, sizeof(_si_remote));
        if(len!= (int)_socketBufferSize) {
        	_isConnected = false;
        }
		_socketBufferSize = 0;
    }
#endif
    _sendPtr=2; // because of start and len
}
void Linker::sendByte(uint8_t data)
{
    data &= 0xff;
    if (data == 0xAA)
    {
        send(0x5A);
        send(0x00);
    }
    else if (data == 0x5A)
    {
        send(0x5A);
        send(0x5A);
    }
    else
    {
        send(data);
    }
}
void Linker::send(uint8_t data)
{
    if (_serialPort != -1)
    {
        size_t len = ::write(_serialPort, &data, 1);
        if(len!=1) {
        	_isConnected = false;
        }
    }
#ifndef __PX4_NUTTX
    if(_socket !=-1 && _socketBufferSize<sizeof(_socketBuffer)){
    	_socketBuffer[_socketBufferSize] = data;
    	_socketBufferSize++;
    }
#endif
}


// ========= Data to Bytes/ Bytes to data
//-Write Byte
void Linker::fillUInt8(uint8_t data)
{
    _sendBuffer[_sendPtr++] = (uint8_t)(data &0xff);
}
void Linker::fillInt8(int8_t data)
{
    _sendBuffer[_sendPtr++] = (int8_t)(data &0xff);
}
void Linker::fillInt16(int16_t data)
{
    _sendBuffer[_sendPtr++] = (int8_t)(data &0xff);
    _sendBuffer[_sendPtr++] = (int8_t)((data >> 8)&0xff);
}
void Linker::fillUInt16(uint16_t data)
{
    _sendBuffer[_sendPtr++] = (uint8_t)(data &0xff);
    _sendBuffer[_sendPtr++] = (uint8_t)((data >> 8)&0xff);
}
void Linker::fillInt32(int32_t data)
{
    _sendBuffer[_sendPtr++] = (int8_t)(data &0xff);
    _sendBuffer[_sendPtr++] = (int8_t)((data >> 8)&0xff);
    _sendBuffer[_sendPtr++] = (int8_t)((data >> 16)&0xff);
    _sendBuffer[_sendPtr++] = (int8_t)((data >> 24)&0xff);
}
void Linker::fillUInt32(uint32_t data)
{
    _sendBuffer[_sendPtr++] = (uint8_t)(data &0xff);
    _sendBuffer[_sendPtr++] = (uint8_t)((data >> 8)&0xff);
    _sendBuffer[_sendPtr++] = (uint8_t)((data >> 16)&0xff);
    _sendBuffer[_sendPtr++] = (uint8_t)((data >> 24)&0xff);
}
/*
void Linker::fillX32(float data)
{
    static unsigned char buffer[4];
    static float* tmp;
    tmp = (float*)&data;
    memcpy(buffer, tmp, 4);

    _sendBuffer[_sendPtr++] = buffer[0];
    _sendBuffer[_sendPtr++] = buffer[1];
    _sendBuffer[_sendPtr++] = buffer[2];
    _sendBuffer[_sendPtr++] = buffer[3];
}

void Linker::X32ToInt8(double data, double scale)
{
    int8_t tmp;
    data /= scale;
    if(data > 255)
    {
        data = 255;
    }
    else if( data < 0.0)
    {
        data = 0.0f;
    }
    tmp = (int8_t) data;
    _sendBuffer[_sendPtr++] = tmp;
}
void Linker::X32ToUInt8(double data, double scale)
{
    uint8_t tmp;
    data /= scale;
    if(data > 255)
    {
        data = 255;
    }
    else if( data < 0.0)
    {
        data = 0.0f;
    }
    tmp = (uint8_t) data;
    _sendBuffer[_sendPtr++] = tmp;
}
void Linker::X32ToInt16(double data, double scale)
{
    int16_t tmp;
    data /= scale;
    if(data > 32767.0)
    {
        data = 32767.0;
    }
    else if(data < -32767.0)
    {
        data = -32767.0;
    }
    tmp = (int)data;
    fillInt16(tmp);

}
void Linker::X32ToUInt16(double data, double scale)
{
    uint16_t tmp;
    data /= scale;
    if(data > 65535.0)
        data = 65535.0;
    else if(data < 0.0)
        data = 0.0;
    tmp = (uint16_t)data;
    fillInt16(tmp);
}
void Linker::X32ToInt32(double data, double scale)
{
    int32_t tmp;
    data /= scale;
    if(data > 2147483647.0)
    {
        data = 2147483647.0;
    }
    else if(data < -2147483648.0)
    {
        data = -2147483648.0;
    }
    tmp = (int32_t)data;
    fillInt32(tmp);
}
void Linker::X32ToUInt32(double data, double scale)
{
    uint32_t tmp;
        data /= scale;
        if(data > 4294967295.0)
        {
            data = 4294967295.0;
        }
        else if(data < 0.0)
        {
            data = 0.0;
        }
        tmp = (uint32_t)data;
        fillUInt32(tmp);
}

//- Read Byte
bool Linker::readBitfromByte(uint8_t *data, int bit)
{
    bool tmp = false;
      switch (bit)
      {
      case 0:
      {
          tmp = (data[0] & 0x01)!=0;
      }
          break;
      case 1:
      {
          tmp = (data[0] & 0x02)!=0;
      }
          break;
      case 2:
      {
          tmp = (data[0] & 0x04)!=0;
      }
          break;
      case 3:
      {
          tmp = (data[0] & 0x08)!=0;
      }
          break;
      case 4:
      {
          tmp = (data[0] & 0x10)!=0;
      }
          break;
      case 5:
      {
          tmp = (data[0] & 0x20)!=0;
      }
          break;
      case 6:
      {
          tmp = (data[0] & 0x40)!=0;
      }
          break;
      case 7:
      {
          tmp = (data[0] & 0x80)!=0;
      }
          break;
      default:
          return false;
          break;
      }
      return tmp;
}
*/
void Linker::readString(uint8_t *data, char* str)
{
    int i = 0;
    while(data[i] != 0 && i<15)
    {
    	str[i]+= data[i];
        i++;
    }
    str[i]=0;
}
/*
int8_t Linker::readInt8(uint8_t *data)
{
    int8_t tmp = data[0];
    return tmp;
}
int16_t Linker::readInt16(uint8_t *data)
{
    return data[0] + (data[1] << 8);
}
uint16_t Linker::readUInt16(uint8_t *data)
{
    return data[0] + (data[1] << 8);
}
int32_t Linker::readInt32(uint8_t *data)
{
    int32_t value = 0;
    for(int i = 0; i<4; i++)
    {
        int32_t tmp = data[i];
        tmp = tmp<<i*8;
        value += tmp;
    }
    return value;
}
uint32_t Linker::readUInt32(uint8_t *data)
{
    uint32_t value = 0;
    for(int i = 0; i<4; i++)
    {
        uint32_t tmp = data[i];
        tmp = tmp<<i*8;
        value += tmp;
    }
    return value;
}
float Linker::readX32(uint8_t *data)
{
    unsigned char tmp[4];
    tmp[0] = data[0];
    tmp[1] = data[1];
    tmp[2] = data[2];
    tmp[3] = data[3];
    float floatReaded;
    memcpy(&floatReaded, tmp, 4);

    return floatReaded;
}
*/
