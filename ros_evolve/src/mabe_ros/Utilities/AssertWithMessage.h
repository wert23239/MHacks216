ros::Rate rate(runRate);
	/* RUN MODE DESCRIPTION: This mode just runs the organism passed to runWorldSolo until ROS dies.
	RUN in SIMULATOR
	*/
	// Each time step consists of the following steps:
	//  1) SENSE, 2) THINK (brain update), 3) ACTUATE, 4) WORLD UPDATE
	// Wait for a laser scan message
	ros::topic::waitForMessage<sensor_msgs::LaserScan>("base_scan", nodeHandle);
	ros::spinOnce();  // We'll want to spin once when we receive it (calls our callbacks for us)
	while (ros::ok()) {
		/////////////////////////////////////////////
		// 1: SENSE: range data
		/////////////////////////////////////////////
		vector<float> filteredRanges = filterLaser(currentLaser);
		cout << "FILTERED RANGES: ";
		for (int i = 0; i < (int)filteredRanges.size(); i++) {
			cout << filteredRanges[i] << " ";
			org->brain->setInput(i, filteredRanges[i]);
		}
		cout << endl;
		/////////////////////////////////////////////
		// 2: UPDATE
		/////////////////////////////////////////////
		org->brain->update();
		/////////////////////////////////////////////
		// 3: ACTUATE
		/////////////////////////////////////////////
	
		/*geometry_msgs::Twist cmdVelMsg;
		cmdVelMsg.linear.x = linearVelocityLookup(linearAction);
		cmdVelMsg.angular.z = angularVelocityLookup(angularAction);*/
		// Send off the command!
		cmdVelPub.publish(org->brain->getOutput(0));
		//cout << "Linear action: " << linearAction << endl;
		//cout << "Angular action: " << angularAction << endl;
		/////////////////////////////////////////////
		// 4: WORLD UPDATE
		/////////////////////////////////////////////
		// Step through time!
		ros::spinOnce();
		rate.sleep();
	};