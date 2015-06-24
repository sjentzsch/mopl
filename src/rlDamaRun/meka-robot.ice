module robot {
	
	// Important: This interface definition is not mature yet and may change often.


	sequence<float> SequenceReal;
	sequence<string> SequenceString;
	
	exception RobotError {
		string reason;
	};

        /**
          Methods to control the Meka robot.
         **/
        interface RobotInterface
        {	

		// almost equivalent to https://mekabot-dev.com/m3doc/html/#m3humanoid-methods
		// with one exceptional difference: get/setTheta* internally call proxy.step()!

		SequenceString getAvailableChains();
	
		int getNumDof(string chain) throws RobotError;

		SequenceReal getThetaDeg(string chain) throws RobotError;

		SequenceReal getThetaDotDeg(string chain) throws RobotError;


		void setStiffness(string chain, SequenceReal stiffness) throws RobotError;

		void setSlewRateProportion(string chain, SequenceReal slew) throws RobotError;

		void setModeThetaGc(string chain) throws RobotError;

		void setThetaDeg(string chain, SequenceReal angles) throws RobotError;

		void proxyStep();


		void helloWorld(string s);
		

		void doHandRelease();
		
		void doHandPushPos();
		
		void doHandGrasp();
		
		void setHandPayload(float mass);
		
        };
};

