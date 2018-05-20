//
// Created by   on 03/05/17.
//

#include "WMDynamixelNode.h"
#include "WMDynamixel.h"

int main(int argc, char **argv){
	return wm_dynamixel::main(argc, argv);
}

namespace wm_dynamixel {
	dynamixel::PortHandler *portHandler;
	dynamixel::PacketHandler *packetHandler;

	ros::Publisher dynamixelPublisher;

	std::vector<WMDynamixel> dynamixelArray;

	int main(int argc, char **argv) {
		ros::init(argc, argv, "wm_dynamixel_node");
		ros::NodeHandle dynamixelHandler;

		//read parameters
		std::string tty = ros::param::param<std::string>("/" + ros::this_node::getName() + "/port_name", PORTNAME);
		auto iBaud = ros::param::param<int>("/" + ros::this_node::getName() + "/baud_rate", BAUDRATE);


		ROS_DEBUG("Will try to open RS485 Initialised on %s with %i bauds!", tty.c_str(), iBaud);

		//initialise port
		for (int iTry = 1; iTry <= NBR_OF_TRY; iTry++) {
			//try initialisation
			bool bInitialised = wm_dynamixel::InitPort(tty.c_str(), iBaud);

			if (!bInitialised && iTry == NBR_OF_TRY) {
				ROS_ERROR("Error initialising RS485 port (Try %i/%i). Process will finish.\n", NBR_OF_TRY, NBR_OF_TRY);
				return 1;
			} else if (!bInitialised) {
				ROS_ERROR("Error initialising RS485 port (Try %i/%i). Waiting 5 seconds until try again.\n", iTry,
				          NBR_OF_TRY);
				sleep(5);
			} else {
				ROS_INFO("RS485 Initialised on %s with %i bauds!", tty.c_str(), iBaud);
				break;
			}
		}

		//initialise ros service for data reading and writing
		ros::ServiceServer service_read = dynamixelHandler.advertiseService("dynamixel/read_addr", wm_dynamixel::Read_Data_Dynamixel);
		ros::ServiceServer service_write = dynamixelHandler.advertiseService("dynamixel/write_addr", wm_dynamixel::Write_Data_Dynamixel);

		//initialise ros subscriber for commands
		ros::Subscriber dynamixelSubscriber = dynamixelHandler.subscribe("dynamixel_cmd", 10, wm_dynamixel::UpdateVelocity);

		//initialise ros subscriber for initialisations
		ros::Subscriber newDynamixelSubscriber = dynamixelHandler.subscribe("dynamixel_init", 10, wm_dynamixel::addDynamixel);

		//initialise ros publisher for data feedback
		wm_dynamixel::dynamixelPublisher = dynamixelHandler.advertise<std_msgs::Float64MultiArray>("dynamixel_pos", 10);

		//run ros loop
		wm_dynamixel::nodeLoop();

		return 0;
	}

	void nodeLoop() {
		ros::Rate loop_rate(10);
		ROS_INFO("Going in node loop.");
		while (ros::ok()) {
			ros::spinOnce();
			for (auto &index : dynamixelArray) {
				index.publishPosition(dynamixelPublisher);
			}
			loop_rate.sleep();
		}
	}

	void UpdateVelocity(std_msgs::Float64MultiArrayConstPtr msg) {
		auto ID = (int) msg->data[0];
		for (auto &index : dynamixelArray) {
			if (index.getID() == ID) {
				index.setCmd(msg->data[1]);

				if (index.getMode() == 0) {
					index.setVelocity(index.getCmd());
				} else if (index.getMode() == 1) {
					index.setPosition(index.getCmd());
				}

				break;
			}
		}
	}

	void addDynamixel(std_msgs::Float64MultiArrayConstPtr msg) {
		auto ID = (int) msg->data[0];
		double offset = msg->data[1];
		auto resolution = (int) msg->data[2];
		auto direction = (int) msg->data[3];

		auto mode = (int) msg->data[4];
		double ratio = msg->data[5];
		auto maxSpeed = (int) msg->data[6];

		ROS_INFO("Try to add a dynamixel with ID %i, offset %f and coef %i.", ID, offset, resolution);

		for (auto &index : dynamixelArray) {
			if (index.getID() == ID) {
				index.updateDynamixel(ID, offset, resolution, direction, mode, ratio, maxSpeed);
				break;
			}
		}
		dynamixelArray.push_back(WMDynamixel(ID, offset, resolution, direction, mode, ratio, maxSpeed));

	}

	bool InitPort(const char *PortName, int BaudRate) {
		// Link port
		portHandler = dynamixel::PortHandler::getPortHandler(PortName);

		// Link packet
		packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);

		// Open port
		if (!portHandler->openPort()) {
			return false;
		}

		// Set port baudrate
		return portHandler->setBaudRate(BaudRate);
	}

	bool write1BDynamixel(int ID, int iAddress, int iValue) {
		int dxl_comm_result;
		uint8_t dxl_error = 0;

		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, (uint8_t) ID, (uint16_t) iAddress,
		                                                (uint8_t) iValue,
		                                                &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) {
			packetHandler->printTxRxResult(dxl_comm_result);
			return false;
		} else if (dxl_error != 0) {
			packetHandler->printRxPacketError(dxl_error);
			return false;
		}

		return true;
	}

	bool write2BDynamixel(int ID, int iAddress, int iValue) {
		int dxl_comm_result;
		uint8_t dxl_error = 0;
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, (uint8_t) ID, (uint16_t) iAddress,
		                                                (uint16_t) iValue,
		                                                &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) {
			packetHandler->printTxRxResult(dxl_comm_result);
			return false;
		} else if (dxl_error != 0) {
			packetHandler->printRxPacketError(dxl_error);
			return false;
		}
		return true;
	}

	int read2BDynamixel(int ID, int iAddress, bool *returnError) {
		int dxl_comm_result;
		uint16_t returnValue;
		uint8_t dxl_error = 0;
		*returnError = false;
		// Read present position
		dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, (uint8_t) ID, (uint16_t) iAddress, &returnValue,
		                                               &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) {
			packetHandler->printTxRxResult(dxl_comm_result);
			*returnError = true;
			return false;
		} else if (dxl_error != 0) {
			packetHandler->printRxPacketError(dxl_error);
			*returnError = true;
			return false;
		}
		return returnValue;
	}

	uint16_t itIsA2BValue(uint16_t address, bool *is2BValue) {
		uint16_t iAddress;

		switch (address) {
			case 6:
			case 7:
				*is2BValue = true;
				iAddress = 6;
				break;

			case 8:
			case 9:
				*is2BValue = true;
				iAddress = 8;
				break;

			case 14:
			case 15:
				*is2BValue = true;
				iAddress = 14;
				break;

			case 20:
			case 21:
				*is2BValue = true;
				iAddress = 20;
				break;

			case 30:
			case 31:
				*is2BValue = true;
				iAddress = 30;
				break;

			case 32:
			case 33:
				*is2BValue = true;
				iAddress = 32;
				break;

			case 34:
			case 35:
				*is2BValue = true;
				iAddress = 34;
				break;

			case 48:
			case 49:
				*is2BValue = true;
				iAddress = 48;
				break;

			case 68:
			case 69:
				*is2BValue = true;
				iAddress = 68;
				break;

			case 71:
			case 72:
				*is2BValue = true;
				iAddress = 71;
				break;

			default:
				*is2BValue = false;
				iAddress = address;
				break;
		}
		return iAddress;
	}

	bool Read_Data_Dynamixel(wm_dynamixel_node::ReadDataDynamixel::Request &req,
	                         wm_dynamixel_node::ReadDataDynamixel::Response &res) {
		int dxl_comm_result;
		uint16_t returnValue16;
		uint8_t returnValue8;
		uint8_t returnError = 0;

		bool bIts2Bytes;
		auto iAddress = itIsA2BValue((uint16_t) req.address, &bIts2Bytes);

		auto ID = (uint8_t) req.id;

		if (bIts2Bytes) {
			dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, ID, iAddress, &returnValue16, &returnError);
			ROS_INFO("result: %i ; value: %i ; error: %i", dxl_comm_result, returnValue16, returnError);
			res.value = returnValue16;
		} else {
			dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, ID, iAddress, &returnValue8, &returnError);
			ROS_INFO("result: %i ; value: %i ; error: %i", dxl_comm_result, returnValue8, returnError);
			res.value = returnValue8;
		}
		res.error = returnError;

		return dxl_comm_result == COMM_SUCCESS;
	}

	bool Write_Data_Dynamixel(wm_dynamixel_node::WriteDataDynamixel::Request &req,
	                          wm_dynamixel_node::WriteDataDynamixel::Response &res) {

		bool bIts2Bytes;
		uint16_t iAddress = itIsA2BValue((uint16_t) req.address, &bIts2Bytes);

		int dxl_comm_result;
		uint8_t returnError = 0;

		auto iID = (uint8_t) req.id;


		if (bIts2Bytes) {
			auto iValue = (uint16_t) req.value;
			dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, iID, iAddress, iValue, &returnError);
		} else {
			auto iValue = (uint8_t) req.value;
			dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, iID, iAddress, iValue, &returnError);
		}

		res.error = returnError;

		return dxl_comm_result == COMM_SUCCESS;
	}
}
