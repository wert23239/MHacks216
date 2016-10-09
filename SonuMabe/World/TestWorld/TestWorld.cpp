//for (int i = 0; i < worldUpdates; ++i)
//{
//	// Field array-wise mechanics
//	int ballIndex;

//	// Flush brain
//	org->brain->resetBrain();

//	// Send all elements of field to brain
//	for (int j = 0; j < field.size(); j++)
//		org->brain->setInput(j, field[j]);

//	// Update (black-box)
//	org->brain->update();

//	// readOutput is the decision whether
//	// to swing

//	// What exactly is readOutput(i)?
//	output = org->brain->readOutput(0);

//	// If brain decides to swing
//	if (output == 1) 
//	{
//		
//		// Score for each 200 / 20 * 100 = 1000
//		score += 1;

//		makeField(); // send another ball
//	}

//	//not swinging
//	else  
//	{
//		// Get the position of the ball
//		for (int n = 0; n < field.size(); n++) {
//			if (field[n] == 1)
//				ballIndex = n;
//		}


//		// If ball is to the right of index 0
//		if (ballIndex > 0)
//		{
//			field[ballIndex - 1] = 1;
//			field[ballIndex] = 0;
//		}

//		// Missed ball goes left of array
//		else if (ballIndex < 0)
//		{
//			makeField();
//			score -= 10000000;
//		}
//		
//		// Ball is at exactly index 0
//		else
//		{
//			// modify field
//			field[ballIndex] = 1;
//			field[ballIndex+1] = 0;

//			// Ball being at index 0 is desirable
//			// Score should produce 
//			// worldUpdates / field.size() * score_increment, or
//			// 200 / 20 * 2000 = 20000
//			score += 0; 
//		}
//	}
//}





////for (int i = 0; i < org->brain->nrOutNodes; i++) {
////	if (mode == 0) {
////		score += Bit(org->brain->readOutput(i));
////	} else {
////		score += org->brain->readOutput(i);
////	}