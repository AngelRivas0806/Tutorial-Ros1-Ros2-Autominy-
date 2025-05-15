#include "ArduinoCommunication.h"
#include "COBS.h"

namespace arduino_communication {

	ArduinoCommunication::ArduinoCommunication(const rclcpp::NodeOptions& opts)
	: rclcpp::Node("arduino_communication", opts)
	{
	    device    = declare_parameter<std::string>("device", "/dev/ttyUSB0");
	    baudrate  = declare_parameter<int>("baud", 115200);

	    try {
		serial = std::make_unique<serial::Serial>(device, baudrate);
	    } catch (const std::exception &e) {
		RCLCPP_FATAL(
		    this->get_logger(),
		    "No pude abrir %s a %d baudios: %s",
		    device.c_str(), baudrate, e.what()
		);
		throw;
	    }
	    if (!serial->isOpen()) {
		RCLCPP_FATAL(
		    this->get_logger(),
		    "Puerto serial %s abierto, pero isOpen()==false",
		    device.c_str()
		);
		throw std::runtime_error("Serial no abierto");
	    }
	    RCLCPP_INFO(
		this->get_logger(),
		"Puerto serial %s abierto a %d baudios",
		device.c_str(), baudrate
	    );

	    steeringAnglePublisher = create_publisher<autominy_msgs::msg::SteeringFeedback>("steering_angle", 1);
	    voltagePublisher       = create_publisher<autominy_msgs::msg::Voltage>("voltage", 1);
	    ticksPublisher         = create_publisher<autominy_msgs::msg::Tick>("ticks", 1);
	    imuPublisher           = create_publisher<sensor_msgs::msg::Imu>("imu", 1);
	    imuTemperaturePublisher = create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 1);

	    speedSubscriber    = create_subscription<autominy_msgs::msg::SpeedPWMCommand>(
		"speed", 1,
		std::bind(&ArduinoCommunication::onSpeedCommand, this, std::placeholders::_1)
	    );
	    steeringSubscriber = create_subscription<autominy_msgs::msg::SteeringPWMCommand>(
		"steering", 1,
		std::bind(&ArduinoCommunication::onSteeringCommand, this, std::placeholders::_1)
	    );
	    ledSubscriber      = create_subscription<std_msgs::msg::String>(
		"led", 1,
		std::bind(&ArduinoCommunication::onLedCommand, this, std::placeholders::_1)
	    );
	    imuCalibrationService = create_service<std_srvs::srv::Empty>(
		"calibrate_imu",
		std::bind(&ArduinoCommunication::calibrateIMU, this, std::placeholders::_1, std::placeholders::_2)
	    );

	    receiveBuffer.resize(512);
	    receiveBufferIndex = 0;
	    timer_ = create_wall_timer(
		std::chrono::milliseconds(10),
		std::bind(&ArduinoCommunication::handleSerialAndHeartbeat, this)
	    );
	}

	void ArduinoCommunication::handleSerialAndHeartbeat() {
	    while (serial->available() > 0) {
		uint8_t buf[128];
		size_t bytes = serial->read(buf, sizeof(buf));
		for (size_t i = 0; i < bytes; ++i) {
		    uint8_t byte = buf[i];
		    if (byte == 0) {
		        std::vector<uint8_t> decodeBuffer(receiveBufferIndex);
		        size_t numDecoded = COBS::decode(
		            receiveBuffer.data(), receiveBufferIndex,
		            decodeBuffer.data()
		        );
		        onReceive(decodeBuffer.data(), numDecoded);
		        receiveBufferIndex = 0;
		    } else {
		        if (receiveBufferIndex < receiveBuffer.size()) {
		            receiveBuffer[receiveBufferIndex++] = byte;
		        } else {
		            RCLCPP_ERROR(get_logger(), "Buffer overflow");
		            receiveBufferIndex = 0;
		        }
		    }
		}
	    }
	    onHeartbeat();
	}

	size_t ArduinoCommunication::onSend(uint8_t* message, size_t length) {
	    try {
		if (serial && serial->isOpen()) {
		    return serial->write(message, length);
		} else {
		    RCLCPP_ERROR(get_logger(), "Serial no abierto, no se puede enviar");
		}
	    } catch (const std::exception &e) {
		RCLCPP_ERROR(get_logger(), "Excepción al enviar: %s", e.what());
	    }
	    return 0;
	}
	void ArduinoCommunication::onSpeedCommand(const autominy_msgs::msg::SpeedPWMCommand::SharedPtr speed) {
	    // Implementación real aquí o mínima para compilar
	}

	void ArduinoCommunication::onSteeringCommand(const autominy_msgs::msg::SteeringPWMCommand::SharedPtr steering) {
	    // Implementación real aquí o mínima para compilar
	}

	void ArduinoCommunication::onLedCommand(const std_msgs::msg::String::SharedPtr led) {
	    // Implementación real aquí o mínima para compilar
	}

	bool ArduinoCommunication::calibrateIMU(
	    const std_srvs::srv::Empty::Request::SharedPtr req,
	    const std_srvs::srv::Empty::Response::SharedPtr resp) {
	    // Implementación real aquí o mínima
	    return true;
	}

	void ArduinoCommunication::onReceive(uint8_t* message, size_t length) {
	    // Implementación real aquí o mínima para compilar
	}

	void ArduinoCommunication::onHeartbeat() {
	    // Implementación real aquí o mínima para compilar
	}
}  


