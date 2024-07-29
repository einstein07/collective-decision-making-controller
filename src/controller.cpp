/*
 * controller.cpp
 *
 *  Created on: 08 Jul 2024
 *  Author: Sindiso Mkhatshwa
 *  Email: sindiso.mkhatshwa@uni-konstanz.de
 */
#include "controller.h"

using namespace std;

using namespace collective_decision_making::msg;
using std::placeholders::_1;

/************************************************/
/* Wheel Turning Parameters
/***********************************************/
void Controller::SWheelTurningParams::Init() {
   /**try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }*/

	TurningMechanism = NO_TURN;
	HardTurnOnAngleThreshold = ToRadians(CDegrees(50));
	SoftTurnOnAngleThreshold = ToRadians(CDegrees(45));
	NoTurnAngleThreshold = ToRadians(CDegrees(30));
	MaxSpeed = 10;
}

/************************************************/
/* Flocking Interaction Parameters
/***********************************************/

void Controller::SFlockingInteractionParams::Init() {
      TargetDistance = 75;
      Gain = 1000;
      Exponent = 2;
}

/************************************************/
/* This function is a generalization of the Lennard-Jones potential
/************************************************/
Real Controller::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

Controller::Controller(std::shared_ptr<rclcpp::Node> node){
	/* Wheel turning */
	m_sWheelTurningParams.Init();
	/* Flocking-related */
	m_sFlockingParams.Init();
	auto ns = node->get_namespace();
	// Create the topic to publish
	stringstream cmdVelTopic;
	cmdVelTopic << ns << "/cmd_vel";
	cmdVelPublisher_ = node -> create_publisher<geometry_msgs::msg::Twist>(cmdVelTopic.str(), 1);

	// Create the subscribers
	stringstream lightListTopic, blobTopic;
	lightListTopic << ns << "/light";
	blobTopic << ns << "/blob";

	lightListSubscriber_ = node -> create_subscription<collective_decision_making::msg::LightList>(
				lightListTopic.str(),
				1,
				std::bind(&Controller::lightCallback, this, _1)
				);

	blobSubscriber_ = node -> create_subscription<collective_decision_making::msg::BlobList>(
			blobTopic.str(),
			1,
			std::bind(&Controller::blobCallback, this, _1)
			);
}

void Controller::lightCallback(const LightList lightList) {
	this -> lightList = lightList;
	cmdVelPublisher_ -> publish (SetWheelSpeedsFromVector(VectorToLight() /*+ FlockingVector()*/));
	this->lightList.n = 0;
	this->blobList.n = 0;

}
void Controller::blobCallback(const BlobList blobList) {
	this -> blobList = blobList;
	//cmdVelPublisher_ -> publish (SetWheelSpeedsFromVector(VectorToLight() + FlockingVector()));

}

CVector2 Controller::VectorToLight(){
	/* Calculate a normalized vector that points to the closest light */
	CVector2 cAccum;
	for(size_t i = 0; i < lightList.n; ++i) {
	  cAccum += CVector2(lightList.lights[i].value, CRadians(lightList.lights[i].angle));
	  //std:cout << "value: " << lightList.lights[i].value << " angle: " <<  lightList.lights[i].angle << std::endl;
	}
	if(cAccum.Length() > 0.0f) {
	  /* Make the vector long as 1/4 of the max speed */
	  cAccum.Normalize();
	  cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
	}

	return cAccum;
}

/****************************************/
/****************************************/

CVector2 Controller::FlockingVector() {
   /* Go through the camera readings to calculate the flocking interaction vector */
   if(blobList.n > 0) {
      CVector2 cAccum;
      Real fLJ;
      size_t unBlobsSeen = 0;

      for(size_t i = 0; i < blobList.n; ++i) {

         /*
          * The camera perceives the light as a yellow blob
          * The robots have their red beacon on
          * So, consider only red blobs
          * In addition: consider only the closest neighbors, to avoid
          * attraction to the farthest ones. Taking 180% of the target
          * distance is a good rule of thumb.
          */
         if(blobList.blobs[i].color == "red" &&
            blobList.blobs[i].distance < m_sFlockingParams.TargetDistance * 1.80f) {
            /*
             * Take the blob distance and angle
             * With the distance, calculate the Lennard-Jones interaction force
             * Form a 2D vector with the interaction force and the angle
             * Sum such vector to the accumulator
             */
            /* Calculate LJ */
            fLJ = m_sFlockingParams.GeneralizedLennardJones(blobList.blobs[i].distance);
            /* Sum to accumulator */
            cAccum += CVector2(	fLJ,
            		CRadians(blobList.blobs[i].angle));
            /* Increment the blobs seen counter */
            ++unBlobsSeen;
         }
      }
      if(unBlobsSeen > 0) {
         /* Divide the accumulator by the number of blobs seen */
         cAccum /= unBlobsSeen;
         /* Clamp the length of the vector to the max speed */
         if(cAccum.Length() > m_sWheelTurningParams.MaxSpeed) {
            cAccum.Normalize();
            cAccum *= m_sWheelTurningParams.MaxSpeed;
         }
         return cAccum;
      }
      else
         return CVector2();
   }
   else {
      return CVector2();
   }
}

geometry_msgs::msg::Twist Controller::SetWheelSpeedsFromVector(const CVector2& c_heading) {

   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   geometry_msgs::msg::Twist twist;
   twist.linear.x = fLeftWheelSpeed;
   twist.linear.y = fRightWheelSpeed;
   //std::cout << "heading angle: " << Abs(cHeadingAngle) <<
	//	   " Wheel turning param: " << m_sWheelTurningParams.TurningMechanism << std::endl;

   return twist;
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("argos_ros_node");
	Controller controller(node);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;

}
