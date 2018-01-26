//
// Created by   on 03/05/17.
//

#include "WMDynamixelNode.h"
#include "WMDynamixel.h"


dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

ros::Publisher dynamixelPublisher;

std::vector<WMDynamixel> dynamixelArray;

int main(int argc, char **argv){
	ros::init(argc, argv, "wm_dynamixel_node");
	ros::NodeHandle dynamixelHandler;

	//read parameters
	std::string tty = ros::param::param<std::string>("/" + ros::this_node::getName() + "/port_name", PORTNAME);
	int iBaud = ros::param::param<int>("/" + ros::this_node::getName() + "/baud_rate", BAUDRATE);


	ROS_DEBUG("Will try to open RS485 Initialised on %s with %i bauds!", tty.c_str(), iBaud);

	//initialise port
	for (int iTry = 1; iTry <= NBR_OF_TRY; iTry++) {
		//try initialisation
		bool bInitialised = InitPort(tty.c_str(), iBaud);

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

	//initialise ros service for data reading
	ros::ServiceServer service = dynamixelHandler.advertiseService("Read_Data_Dynamixel", Read_Data_Dynamixel);

	//initialise ros subscriber for commands
	ros::Subscriber dynamixelSubscriber = dynamixelHandler.subscribe("dynamixel_cmd", 10, UpdateVelocity);

	//initialise ros subscriber for initialisations
	ros::Subscriber newDynamixelSubscriber = dynamixelHandler.subscribe("dynamixel_init", 10, addDynamixel);

	//initialise ros publisher for data feedback
	dynamixelPublisher = dynamixelHandler.advertise<std_msgs::Float64MultiArray>("dynamixel_pos", 10);

	//run ros loop
	nodeLoop();

	return 0;
}

void nodeLoop() {
	ros::Rate loop_rate(10);
	int iCount = 0;
	ROS_INFO("Going in node loop.");
	while(ros::ok()){
		//ROS_INFO("Looping");
		ros::spinOnce();
		WriteVelocity();
		for (int index = 0; index < dynamixelArray.size(); index++) {
			dynamixelArray[index].publishPosition(dynamixelPublisher);
		}
		iCount ++;
		loop_rate.sleep();
	}
}

void UpdateVelocity(std_msgs::Float64MultiArrayConstPtr msg) {
	int ID = (int)msg->data[0];
	for (int index=0; index < dynamixelArray.size(); index++) {
		if(dynamixelArray[index].getID() == ID){
			dynamixelArray[index]._cmd = msg->data[1];
			break;
		}
	}
}

void WriteVelocity() {
	for (int index=0; index < dynamixelArray.size(); index++) {
		if ( dynamixelArray[index]._mode == 0 ){
			dynamixelArray[index].setVelocity(dynamixelArray[index]._cmd);
		} else if ( dynamixelArray[index]._mode == 1 ) {
			dynamixelArray[index].setPosition(dynamixelArray[index]._cmd);
		}
	}
}

void addDynamixel(std_msgs::Float64MultiArrayConstPtr msg) {
	int ID = (int)msg->data[0];
	double offset = msg->data[1];
	int resolution = (int)msg->data[2];
	int direction = (int)msg->data[3];

	int mode = (int)msg->data[4];
    double ratio = msg->data[5];
    int maxSpeed = (int)msg->data[6];


	ROS_INFO("Try to add a dynamixel with ID %i, offset %f and coef %i.",ID,offset,resolution);

	for (int index=0; index < dynamixelArray.size(); index++) {
		if(dynamixelArray[index].getID() == ID){
			//ROS_INFO("Will update a dynamixel with ID %i, offset %f and coef %i.",ID,offset,resolution);

			dynamixelArray[index].updateDynamixel(ID, offset, resolution, direction, mode, ratio, maxSpeed);

			break;
		}
	}

	//ROS_INFO("Will add a dynamixel with ID %i, offset %f and coef %i.",ID,offset,resolution);

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
	if (!portHandler->setBaudRate(BaudRate)) {
		return false;
	}
	return true;
}

bool write1BDynamixel(int ID, int iAddress, int iValue){
	int dxl_comm_result;
	uint8_t dxl_error = 0;

	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID, iAddress, iValue, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
		return false;
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
		return false;
	}

	return true;
}

bool write2BDynamixel(int ID, int iAddress, int iValue){
	int dxl_comm_result;
	uint8_t dxl_error = 0;
	dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, ID, iAddress, iValue, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
		return false;
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
		return false;
	}
	return true;
}

int read1BDynamixel(int ID, int iAddress) {
	int dxl_comm_result;
	uint8_t dxl_error = 0;

	uint16_t dxl_present_position;
	// Read present position
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, ID, iAddress, &dxl_present_position, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
		return false;
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
		return false;
	}
	return dxl_present_position;
}

int read2BDynamixel(int ID, int iAddress, bool *returnError) {
	int dxl_comm_result;
	uint16_t returnValue;
	uint8_t dxl_error = 0;
	*returnError = false;
	// Read present position
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, ID, iAddress, &returnValue, &dxl_error);
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

bool Read_Data_Dynamixel(wm_dynamixel_node::ReadDataDynamixel::Request &req,
                         wm_dynamixel_node::ReadDataDynamixel::Response &res) {
	bool returnResult;
	res.value = read2BDynamixel(req.id, req.address, &returnResult);
	return returnResult;
}

