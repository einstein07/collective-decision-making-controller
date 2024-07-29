/*
 * controller.h
 *
 *  Created on: 08 Jul 2024
 *  Author: Sindiso Mkhatshwa
 *  Email: sindiso.mkhatshwa@uni-konstanz.de
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <string>
#include <chrono>
#include <memory>
#include <iostream>
#include <sstream>

/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "collective_decision_making/msg/light.hpp"
#include "collective_decision_making/msg/light_list.hpp"
#include "collective_decision_making/msg/blob.hpp"
#include "collective_decision_making/msg/blob_list.hpp"

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

using namespace argos;
using namespace collective_decision_making::msg;

class Controller{
public:
	/*
	* The following variables are used as parameters for
	* turning during navigation. You can set their value
	* in the <parameters> section of the XML configuration
	* file, under the
	* <controllers><footbot_flocking_controller><parameters><wheel_turning>
	* section.
	*/
   struct SWheelTurningParams {
	  /*
	   * The turning mechanism.
	   * The robot can be in three different turning states.
	   */
	  enum ETurningMechanism
	  {
		 NO_TURN = 0, // go straight
		 SOFT_TURN,   // both wheels are turning forwards, but at different speeds
		 HARD_TURN    // wheels are turning with opposite speeds
	  } TurningMechanism;
	  /*
	   * Angular thresholds to change turning state.
	   */
	  CRadians HardTurnOnAngleThreshold;
	  CRadians SoftTurnOnAngleThreshold;
	  CRadians NoTurnAngleThreshold;
	  /* Maximum wheel speed */
	  Real MaxSpeed;

	  void Init(/**TConfigurationNode& t_tree*/);
   };
   /*
	* The following variables are used as parameters for
	* flocking interaction. You can set their value
	* in the <parameters> section of the XML configuration
	* file, under the
	* <controllers><footbot_flocking_controller><parameters><flocking>
	* section.
	*/
	struct SFlockingInteractionParams {
	 /* Target robot-robot distance in cm */
	 Real TargetDistance;
	 /* Gain of the Lennard-Jones potential */
	 Real Gain;
	 /* Exponent of the Lennard-Jones potential */
	 Real Exponent;

	 void Init(/**TConfigurationNode& t_node*/);
	 Real GeneralizedLennardJones(Real f_distance);
	};
public:

	/** Class constructor */
   Controller(std::shared_ptr<rclcpp::Node> node);

   /* Class destructor. */
   virtual ~Controller() {}

   void lightCallback(const LightList lightList);

   void blobCallback(const BlobList blobList);

   /*
	* Calculates the vector to the closest light.
	*/
	CVector2 VectorToLight();

	/*
	* Calculates the flocking interaction vector.
	*/
	CVector2 FlockingVector();

   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
	geometry_msgs::msg::Twist SetWheelSpeedsFromVector(const CVector2& c_heading);

private:
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
	// Light list subscriber
	rclcpp::Subscription<collective_decision_making::msg::LightList>::SharedPtr lightListSubscriber_;
	// colored-blob-omnidirectional-camera sensor subscriber
	rclcpp::Subscription<collective_decision_making::msg::BlobList>::SharedPtr blobSubscriber_;
	/* The turning parameters. */
	SWheelTurningParams m_sWheelTurningParams;
	/* The flocking interaction parameters. */
	SFlockingInteractionParams m_sFlockingParams;
	LightList lightList;
	BlobList blobList;

};





#endif /* CONTROLLER_H_ */
