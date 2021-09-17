#ifndef LINKER_H
#define LINKER_H

#include <stdint.h>
#include <math.h>


#ifndef __PX4_NUTTX
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#else
#include <px4_defines.h>
#include <matrix/math.hpp>
#endif

#ifndef M_PI_F
#define M_PI_F			3.14159265f
#endif

//  Class GEN
#define GENPAYLOAD          0x00
//Associated blocs -v-
//**Commands
#define B_MODE              0x00
#define B_VELOCITY          0x01
#define B_POSITION          0x02
#define B_GEOTRACKING       0x03
#define B_NUC               0x05
#define B_VELOFFSET         0x06
#define B_ALIM              0x07
#define B_LASER             0x10
#define B_DZOOMVEL          0x15
#define B_DZOOMPOS          0x16
#define B_AUTOFOCUS         0x17
#define B_MANUALFOCUS       0x18
#define B_ICR               0x19
#define B_DSONYFEATURES     0x1A
#define B_VIDEOSWITCH       0x26
#define B_NAVCTRL           0x27
#define B_IRPOLARITY        0x30
#define B_IZOOMVEL          0x31
#define B_NAVINFO1          0x36
#define B_NAVINFO2          0x37
#define B_GPSTIME           0x3A
#define B_GROUNDMLS         0x3B
#define B_REQCMD            0x40
//**Status
#define B_STATUS            0x80
#define B_QUATERNION        0x81
#define B_TEMPERATURE       0x82
#define B_EULERUAV          0x83
#define B_LLHUAV            0x84
#define B_LLHTARGET         0x85
#define B_LLHTARGETREF      0x86
#define B_UTCDATE           0x87
#define B_UTCTIME           0x88
#define B_SOFTMAIN          0x91
#define B_PN                0x94
#define B_SN                0x95
#define B_MODELCAMDAY       0x96
#define B_MODELCAMIR        0x97
#define B_MODELLASER        0x98
#define B_QUARKSTATUS       0x99
#define B_ACK               0x9A
#define B_GIMBALCONFIG      0x9B


//  Class LRF
#define LRFPAYLOAD          0x04
//Associated blocs -v-
//**Commands
#define B_TRIGLRF           0x00
//**Status
#define B_LRFRESULT         0x80


//Linker class is handeling the receiving of messages from the gimbal and the sending of messages to the gimbal.
class Linker
{

public:
	struct MERIO_MODE{
		typedef enum {
			VELOCITY=0,
			POSITION=1,
			STOW=2,
			GEO_TRACK=3
		}Type;
	};

	struct MERIO_STATE{
		typedef enum {
			INIT=0,
			INDEX,
			WAIT,
			ZEROS,
			VELOCITY,
			POSITION,
			STOW,
			GEOTRACK,
			NUC,
			FACTROY,
			CONFIG,
			MOTOR_PROBLEM
		}Type;
	};
    static const float VEL_MAX_RAD_S;

    explicit Linker();
    /**
     * @brief sendProtocol
     * This method is to be called each time a message need to be send to the gimbal
     */
    void sendProtocol();

    /**
     * @brief addBloc allow to add a bloc to the sending list
     * @param blocClass  -- Id of the Bloc's class (see list of those IDs communiaction protocol)
     * @param blocId    -- Id of the Bloc(see list of those IDs in communiaction protocol)
     */
    void addBloc(uint8_t blocClass, uint8_t blocId);
    uint8_t addLEN();
    bool connectWith(int fd);

#ifndef __PX4_NUTTX
    int _socket = -1;
	struct sockaddr_in _si_remote;
    uint8_t  _socketBuffer[1500]; // tx buffer
    size_t	_socketBufferSize = 0;

    bool connectTo(const char* server, int port, bool blocking) {
    	struct sockaddr_in si_me;
    	int s;

    	//create a UDP socket
    	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    	{
    		perror("socket");
        	return false;
    	}

    	// zero out the structure
    	memset((char *) &si_me, 0, sizeof(si_me));

    	si_me.sin_family = AF_INET;
    	si_me.sin_port = htons(port);
    	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
    	if (!blocking){
    	  int flags = fcntl(s, F_GETFL, 0);
    	  fcntl(s, F_SETFL, flags | O_NONBLOCK);
    	}

    	//bind socket to port
    	if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
    	{
    		perror("bind");
        	return false;
    	}

    	memset((char *) &_si_remote, 0, sizeof(_si_remote));
    	_si_remote.sin_family = AF_INET;
    	_si_remote.sin_port = htons(port);

    	if (inet_aton(server , &_si_remote.sin_addr) == 0)
    	{
    		fprintf(stderr, "inet_aton() failed\n");
        	return false;
    	}

    	_socket = s;
    	return true;
    };

    bool poll(){
    	char buf[1500];
		struct sockaddr_in si_rx;
		socklen_t slen = sizeof(si_rx);
		//try to receive some data
		int res = recvfrom(_socket, buf, sizeof(buf), 0, (struct sockaddr *) &si_rx, &slen);
		if (res <=0)
		{
			return false;
		}
		for(int i=0;i<res;i++){
			receive(buf[i]);
		}
		return true;
    }

#endif

    bool getConnectStatus();
    /* reset flag after reading */
    bool getBlocReceivedClear(uint8_t id) {
    	bool result = false;
    	result = _recFlags[id>>3] & (1<<(id%8)); // get flag
    	_recFlags[id>>3] &= ~(1<<(id%8)); // reset flag;
    	return result;
    };
    /* received message and not clear flag */
    bool getBlocReceived(uint8_t id) const{
    	bool result = false;
    	result = _recFlags[id>>3] & (1<<(id%8)); // get flag
    	return result;
    };

    void setMode(uint8_t merio_mode){
		addBloc(GENPAYLOAD, B_MODE);
		fillUInt8(merio_mode);
		addLEN();
		sendProtocol();
    };

    void setGeoPoint(float lat_deg, float lon_deg, float alt_m){
		addBloc(GENPAYLOAD, B_GEOTRACKING);
		fillInt32((int32_t)(lat_deg*1e9f*M_PI_F/180.0f)); // in rad*1e8
		fillInt32((int32_t)(lon_deg*1e9f*M_PI_F/180.0f)); // in rad*1e8
		fillInt32((int32_t)(alt_m*1e2f));
		addLEN();
		sendProtocol();
    }
    void setPosition(float azi_rad, float ele_rad, float vel_rad_s = VEL_MAX_RAD_S){
    	if(vel_rad_s <= 0.0f) {
    		vel_rad_s = VEL_MAX_RAD_S;
    	}
    	if(vel_rad_s >VEL_MAX_RAD_S) {
    		vel_rad_s = VEL_MAX_RAD_S;
    	}
		addBloc(GENPAYLOAD, B_POSITION);
		fillInt16((int16_t)(wrap_pi(azi_rad)*1e4f));
		fillInt16((int16_t)(wrap_pi(ele_rad)*1e4f));
		fillInt16((int16_t)(vel_rad_s*7776.0f)); // 7776=6^5
		addLEN();
		sendProtocol();
    };

    void setVelocity(float azi_rad_s, float ele_rad_s, float timeout_s = 1.0f){
    	if(azi_rad_s <-VEL_MAX_RAD_S) {
    		azi_rad_s = -VEL_MAX_RAD_S;
    	}
    	if(azi_rad_s > VEL_MAX_RAD_S) {
    		azi_rad_s = VEL_MAX_RAD_S;
    	}
    	if(ele_rad_s <-VEL_MAX_RAD_S) {
    		ele_rad_s = -VEL_MAX_RAD_S;
    	}
    	if(ele_rad_s > VEL_MAX_RAD_S) {
    		ele_rad_s = VEL_MAX_RAD_S;
    	}
		addBloc(GENPAYLOAD, B_VELOCITY);
		fillInt16((int16_t)(azi_rad_s*7776.0f));
		fillInt16((int16_t)(ele_rad_s*7776.0f));
		fillUInt16((uint16_t)(timeout_s*1e2f));
		uint8_t flag = 1;
		/*
		 * Bit 0: prop. Zoom Bit 1-2: expo. Bit 3-7: unused
		 * Used to control the velocity for each axis. The maximum velocity is set to 1.966 rad/s,
		 * commands will be applied during the time timeout.
		 * Prop. Zoom: when set, the velocity command is divided by the zoom ratio.
		 * Expo: decrease command sensitivity around neutral position (0, off).
		 */
		fillUInt8(flag);
		addLEN();
		sendProtocol();
    };

    /*
     * positive is zoom in, reduce FOV
     */
    void setZoomVel(float vel=0.0f){
    	if(vel <-100) {
    		vel = -100;
    	}
    	if(vel > 100) {
    		vel = 100;
    	}
		addBloc(GENPAYLOAD, B_DZOOMVEL);
		fillInt8((int8_t)vel);
		addLEN();
		sendProtocol();
		/*
		 * Used to set the zoom speed of the ir camera, ZOOMVEL must be between -100 and +100.
		 * Set a positive value to reduce the field of view. If ir zoom auto is set to 1, the IR fov will be
		 * controlled by the fov of the day camera.
		 */
		/*addBloc(GENPAYLOAD, B_IZOOMVEL);
		fillInt8((int8_t)vel);
		fillUInt8(1);
		addLEN();
		sendProtocol();*/
    };

    void doNuc(){
		addBloc(GENPAYLOAD, B_NUC);
		addLEN();
		sendProtocol();
    };

    void setGroundAltitude(float alt_msl){
		addBloc(GENPAYLOAD, B_GROUNDMLS);
		fillUInt32((int32_t)(alt_msl*1e2f));
		addLEN();
		sendProtocol();
    }

#pragma pack(push,1)

    typedef struct{
    	uint8_t LEN;
    	uint8_t CLASS;
    	uint8_t ID;
    }Header;

    typedef struct{
    	int16_t  pan_rad; // 1e-4 rad
    	int16_t  tilt_rad; // 1e-4 rad
    	uint16_t fov_day_rad; // 1e-4 rad
    	uint16_t fov_ir_rad; // 1e-4 rad
    	uint8_t  zoom_level; // % zoom level
    	uint8_t  status1; // Bit 0-4 : Gimbal state  	Bit 5-7 : Gimbal mode
    	/*
    	 * Gimbal state:
 0, INIT
 1, INDEX
 2, WAIT
 3, ZEROS
 4, VELOCITY
 5, POSITION
 6, STOW
 7, GEOTRACK
 8, NUC
 9, FACTORY
 10, CONFIG
 11, MOTOR PROBLEM
Gimbal mode:
 0, VELOCITY
 1, POSITION
 2, STOW
 3, GEOTRACK
    	 */
    	uint8_t	 status2;
/*
Bit 0: valid
Bit 1: EO Gyroscopes com.
Bit 2: tilt board com.
Bit 3: pan board com.
Bit 4: tilt index
Bit 5: pan index
Bit 6-7: unused
*/
    	uint8_t status3;
/*
Bit 0: Tilt motor problem
Bit 1: Pan motor problem
Bit 2: Day camera com.
Bit 3: Ir camera com.
Bit 4: Autofocus
Bit 5: ICR
Bit 6: Video Switch
Bit 7: Laser
*/
    	uint8_t status4;
    	/*
    	 *Bit 0: Bias filter
Bit 1: Bias filter simulation
Bit 2: ir zoom locked
Bit 3: LRF link
Bit 4: DCam Wide Dyn range
Bit 5: DCam stabDay
Bit 6: alim day camera
Bit 7: alim Ir camera
    	 */
    	uint8_t navigation_mode;
    }Status;

    typedef struct{
    	//T1: Electro-Optic bloc temperature  . T2: Main electronic board temperature.
    	int16_t TemperatureDay;  // 1e-2 C
    	int16_t TemperatureElectronics;  // 1e-2 C
    }Temperature;

    typedef struct{
    	int32_t	lat_rad; //1e-9 rad
    	int32_t	lon_rad; //1e-9 rad
    	int32_t alt_m; // 1e-2 m MSL
    }LLHTarget;

    typedef struct{
    	int32_t	lat_rad; //1e-9 rad
    	int32_t	lon_rad; //1e-9 rad
    	int32_t alt_m; // 1e-2 m MSL
    	uint8_t sats; // number
    	uint8_t status; // Bit 0 : link		Bit1 : valid		Bit 2-7 : unused
    }LLHuav;
    typedef struct{
    	uint8_t day_model; // 0, no camera 1, EV7500 2, EV7520A 3, EH3150 4, MP1010
    	uint8_t day_format; // 0, 1080p30 1, 1080p25 2, 720p60 3, 720p50 4, 1080i60 5, 1080i50
    	uint8_t gimbal_model; // 0, TEMIS 1, TEMIS XL 2, TEMIS XL14 3, TEMIS XL16 4, TEMIS XL18 5, TEMIS XL19
    	uint8_t laser_model;
    	uint8_t ircam1_model; // 0, no camera 1, Quark 2, ASTIR 3, SMARTIR 4, MICROCAM3 5, NOXANT
    	uint8_t ircam2_model;
    }GimbalConfig;

    typedef struct{
    	int16_t	roll_rad4; // rad 1e-4
    	int16_t	pitch_rad4; // rad 1e-4
    	int16_t	yaw_rad4; // rad 1e-4
    }EulerUav;

#pragma pack(pop)

    Status 		_status{0};
    Temperature _temperature{0};
    LLHTarget	_target{0};
    LLHTarget	_target_ref{0};
    LLHuav		_llh_uav{0}; // feedback from gimbal
    EulerUav		_euler_uav{0}; // feedback from gimbal
    GimbalConfig _config{0};
    char	_SWmain[16];
    char _PN[16];
    char _SN[16];
//    char _cameraDay[16];
//    char _cameraIR[16];
    int	_errors_crc = 0;

    /**
     * @brief receive
     * @param data
     * This method received Bytes one by one, it translate the start bytes
     * (see document TSC-B004 for more precision).
     * And it send them to the decode method.
     */
    void receive(uint8_t data);
    /**
     * @brief handleReadyRead
     * This slot is called by the readyRead() signal from QSerialPort
     * It get the whole bytes received and send them to the receive method
     */
    //void handleReadyRead();
    //void handleError(QSerialPort::SerialPortError serialPortError);
    //void handleBytesWritten(qint64 bytes);
    //void disconnectWith();

    //-Write Byte
        void fillUInt8(uint8_t data);
        void fillInt8(int8_t data);
        void fillInt16(int16_t data);
        void fillUInt16(uint16_t data);
        void fillInt32(int32_t data);
        void fillUInt32(uint32_t data);
        void fillX32(float data);

        void X32ToInt8(double data, double scale);
        void X32ToUInt8(double data, double scale);
        void X32ToInt16(double data, double scale);
        void X32ToUInt16(double data, double scale);
        void X32ToInt32(double data, double scale);
        void X32ToUInt32(double data, double scale);
private:
    enum State {
        START = 0,
        LEN = 1,
        DATA = 2,
        CHECK = 3
    };
    int _serialPort = -1;

    uint8_t _recFlags[32]{0};
    void setBlocReceived(uint8_t id) {
    	_recFlags[id>>3] |= (1<<(id%8)); // set flag;
    };

    State _state = START;
    uint8_t  _x5Adetected = 0;
    uint8_t  _len, _lenPos;
    uint8_t  _ptr=0; // rx buffer pointer
    uint8_t  _sendPtr = 0; //tx buffer pointer
    uint8_t  _buffer[128]; // rx buffer
    uint8_t  _sendBuffer[64]; // tx buffer
    uint8_t _checkFlag = 0;
    uint16_t _checksum;

    bool _isReading = false ;
    bool _isConnected = false;

    void addBlocGENPAYLOAD(uint8_t blocId);
    void decode(uint8_t data, int16_t status);
    /**
     * @brief CRC16
     * @param buffer
     * @param len
     * @param crc
     * @return
     * This method compute the checksum given received message and return it
     * initCRCtab must be called once before to be able to compute the checksum
     * correctly.
     */

    static const 	uint16_t _CRCtab[256];
    uint16_t CRC16(uint8_t *buffer, uint16_t len, uint16_t crc);
    void initCRCtab();

    /**
     * @brief reader
     * @param data
     * @param len
     * This method identify blocs in the message and read them one by one using
     * unbloc method
     */
    void reader(uint8_t *data, int16_t len);
    /**
     * @brief unbloc
     * @param buffer
     * This method read the content of a message bloc
     */
    void unbloc(uint8_t *buffer);
    /**
     * @brief sendByte
     * @param data
     * sendByte is the byteStuffer
     */
    void sendByte(uint8_t data);
    /**
     * @brief send
     * @param data
     * This method uses the write method of QSerialPort to send bytes to the gimbal
     */
    void send(uint8_t data);

//- Read Byte
    bool    readBitfromByte(uint8_t *data, int bit);
    void readString(uint8_t *data, char* str);
    int8_t  readInt8(uint8_t *data);
    int16_t readInt16(uint8_t *data);
    uint16_t readUInt16(uint8_t *data);
    int32_t readInt32(uint8_t *data);
    uint32_t readUInt32(uint8_t *data);
    float readX32(uint8_t *data);

    template<typename Type>
    bool is_finite(Type x) {
    #if defined (__PX4_NUTTX)
        return PX4_ISFINITE(x);
    #elif defined (__PX4_QURT)
        return __builtin_isfinite(x);
    #else
        return std::isfinite(x);
    #endif
    }

    template<typename Type>
    Type wrap_pi(Type x)
    {
        if (!is_finite(x)) {
            return x;
        }

        int c = 0;

        while (x >= Type(M_PI)) {
            x -= Type(2 * M_PI);

            if (c++ > 100) {
                return INFINITY;
            }
        }

        c = 0;

        while (x < Type(-M_PI)) {
            x += Type(2 * M_PI);

            if (c++ > 100) {
                return INFINITY;
            }
        }

        return x;
    }


};

#endif // LINKER_H
