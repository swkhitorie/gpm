#include "mavlink_vehicle.h"

__weak void mavlink_log_erase_all() {}
__weak void mavlink_log_request_list(mavlink_log_request_list_t *prlist_t) {}
__weak void mavlink_log_request_data(mavlink_log_request_data_t *prdata_t) {}

/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
/// This needs to be packed, because it's typecasted from mavlink_file_transfer_protocol_t.payload, which starts
/// at a 3 byte offset, causing an unaligned access to seq_number and offset
struct __attribute__((__packed__)) PayloadHeader {
    uint16_t	seq_number;	///< sequence number for message
    uint8_t		session;	///< Session id for read and write commands
    uint8_t		opcode;		///< Command opcode
    uint8_t		size;		///< Size of data
    uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
    uint8_t		burst_complete; ///< Only used if req_opcode=kCmdBurstReadFile - 1: set of burst packets complete, 0: More burst packets coming.
    uint8_t		padding;        ///< 32 bit aligment padding
    uint32_t	offset;		///< Offsets for List and Read commands
    uint8_t		data[];		///< command data, varies by Opcode
};

/// @brief Command opcodes
enum Opcode : uint8_t {
    kCmdNone,		///< ignored, always acked
    kCmdTerminateSession,	///< Terminates open Read session
    kCmdResetSessions,	///< Terminates all open Read sessions
    kCmdListDirectory,	///< List files in <path> from <offset>
    kCmdOpenFileRO,		///< Opens file at <path> for reading, returns <session>
    kCmdReadFile,		///< Reads <size> bytes from <offset> in <session>
    kCmdCreateFile,		///< Creates file at <path> for writing, returns <session>
    kCmdWriteFile,		///< Writes <size> bytes to <offset> in <session>
    kCmdRemoveFile,		///< Remove file at <path>
    kCmdCreateDirectory,	///< Creates directory at <path>
    kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty
    kCmdOpenFileWO,		///< Opens file at <path> for writing, returns <session>
    kCmdTruncateFile,	///< Truncate file at <path> to <offset> length
    kCmdRename,		///< Rename <path1> to <path2>
    kCmdCalcFileCRC32,	///< Calculate CRC32 for file at <path>
    kCmdBurstReadFile,	///< Burst download session file

    kRspAck = 128,		///< Ack response
    kRspNak			///< Nak response
};

/// @brief Error codes returned in Nak response PayloadHeader.data[0].
enum ErrorCode : uint8_t {
    kErrNone,
    kErrFail,			///< Unknown failure
    kErrFailErrno,			///< Command failed, errno sent back in PayloadHeader.data[1]
    kErrInvalidDataSize,		///< PayloadHeader.size is invalid
    kErrInvalidSession,		///< Session is not currently open
    kErrNoSessionsAvailable,	///< All available Sessions in use
    kErrEOF,			///< Offset past end of file for List and Read commands
    kErrUnknownCommand,		///< Unknown command opcode
    kErrFailFileExists,		///< File/directory exists already
    kErrFailFileProtected,		///< File/directory is write protected
    kErrFileNotFound                ///< File/directory not found
};
    
void MavlinkVehicle::handle_msg_ftp(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_file_transfer_protocol_t ftp_t;
    mavlink_msg_file_transfer_protocol_decode(&_rrxmsg, &ftp_t);  
    PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_t.payload[0]);
    
	switch (payload->opcode) {
	case kCmdNone:
		break;
	case kCmdTerminateSession:
        mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdTerminateSession");
		break;
	case kCmdResetSessions:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdResetSessions");
		break;
	case kCmdListDirectory:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdListDirectory");
		break;
	case kCmdOpenFileRO:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdOpenFileRO");
		break;
	case kCmdCreateFile:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdCreateFile");
		break;
	case kCmdOpenFileWO:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdOpenFileWO");
		break;
	case kCmdReadFile:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdReadFile");
		break;
	case kCmdBurstReadFile:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdBurstReadFile");
		break;
	case kCmdWriteFile:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdWriteFile");
		break;
	case kCmdRemoveFile:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdRemoveFile");
		break;
	case kCmdRename:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdRename");
		break;
	case kCmdTruncateFile:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdTruncateFile");
		break;
	case kCmdCreateDirectory:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdCreateDirectory");
		break;
	case kCmdRemoveDirectory:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdRemoveDirectory");
		break;
	case kCmdCalcFileCRC32:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: kCmdCalcFileCRC32");
		break;
	default:
		mavlink_log_info(MAVLINK_COMM_1, "FTP: unkown ftp opcode");
		break;
	}

    payload->seq_number++;

    if (false) {
        payload->req_opcode = payload->opcode;
        payload->opcode = kRspAck;
    } else {
        uint8_t errorCode = kErrFileNotFound; //kErrFailFileExists kErrFailErrno kErrFileNotFound
        int r_errno = 0;
        payload->req_opcode = payload->opcode;
        payload->opcode = kRspNak;
        payload->size = 1;
        payload->data[0] = errorCode;
        if (errorCode == kErrFailErrno) {
            payload->size = 2;
            payload->data[1] = r_errno;
        }
    }

    ftp_t.target_network = 0;
    ftp_t.target_system = _rrxmsg.sysid;
    ftp_t.target_component = _rrxmsg.compid;
    mavlink_msg_file_transfer_protocol_send_struct(chan, &ftp_t);
}

void MavlinkVehicle::handle_msg_log_request_list(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_log_request_list_t lreq_t;
    mavlink_msg_log_request_list_decode(&_rrxmsg, &lreq_t);
    
    mavlink_log_request_list(this, &lreq_t);
}

void MavlinkVehicle::handle_msg_log_erase_all(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_log_erase_t ler_t; 
    mavlink_msg_log_erase_decode(&_rrxmsg, &ler_t);
    
    mavlink_log_erase_all(this);
}

void MavlinkVehicle::handle_msg_log_request_data(mavlink_message_t *msg)
{
    mavlink_message_t _rrxmsg = *msg;
    mavlink_log_request_data_t lreq_d; 
    mavlink_msg_log_request_data_decode(&_rrxmsg, &lreq_d);
    
    mavlink_log_request_data(this, &lreq_d); 
}


