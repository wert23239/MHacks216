//  MABE is a product of The Hintze Lab @ MSU
//     for general research information:
//         hintzelab.msu.edu
//     for MABE documentation:
//         github.com/ahnt/MABE/wiki
//
//  Copyright (c) 2015 Michigan State University. All rights reserved.
//     to view the full license, visit:
//         github.com/ahnt/MABE/wiki/License

#include "LoIWorld.h"
#include <algorithm>    // std::iter_swap

using namespace std;

shared_ptr<ParameterLink<int>> LoIWorld::numberOfInputsPL = Parameters::register_parameter("WORLD_LoI-numberOfInputs", 30, "number of inputs in this world");
shared_ptr<ParameterLink<int>> LoIWorld::worldUpdatesPL= Parameters::register_parameter("WORLD_LoI-worldUpdates", 100, "number of updates in this world");



void LoIWorld::makeField() {


	field.clear();

	for (int i = 0; i<numberOfInputs; ++i) {
		field.push_back(0);
	}

	// Set baseball end to 1
	field[field.size() - 1] = 1;

	//set ball location for the end of field vector
	ball = field.size() - 1;

}


LoIWorld::LoIWorld(shared_ptr<ParametersTable> _PT) :
	AbstractWorld(_PT) {
	numberOfInputs = (PT == nullptr) ? numberOfInputsPL->lookup() : PT->lookupInt("WORLD_LoI-numberOfInputs");
	worldUpdates = (PT == nullptr) ? worldUpdatesPL->lookup() : PT->lookupInt("WORLD_LoI-worldUpdates");
        
        
}

// score is number of outputs set to 1 (i.e. output > 0) squared
void LoIWorld::runWorldSolo(shared_ptr<Organism> org, bool analyse, bool visualize, bool debug) {

	//initalize the world
	makeField();
	double score = 0.0;
	double output = 0;
    int count = 0;
    vector<int> speed = {1, 2, 3};
    
    //while(!speed.empty()){
    for(int k = 0; k < speed.size(); k++)
    {
        for (int i = 0; i < worldUpdates; ++i)
            {
                org->brain->resetBrain();
                for (int j = 0; j < field.size(); ++j)
                {
                    org->brain->setInput(j, field[j]);
                }
                
                org->brain->update();
                output = org->brain->readOutput(0);


                if (output == 1) 
                {	
                    score += 1;
                    
                    // if the output and the ball connect -- increase fitness score
                    if (field[(field.size() - 1) % speed[0]] == 1)
                    {
                        score += 100000000000;
                    }
                    
                    // if the output is a one and is far from the ball -- decrease the fitness
                    if (ball < 5)
                    {
                        score += 10000 * (field.size() - ball);
                    }
                    
                    // reset the field
                    makeField();	

                }
                
                // if the output is zero
                else 
                {
                    // the last connect point does not equal 1
                    if ( ball < speed[0])
                    {
                        iter_swap(field.begin() + ball, field.begin() + (ball - speed[0]));
                        ball = ball - speed[0];
                        
                    }
                    // the last connect point was reached but contact was not made
                    else {
                        makeField(); 
                        score -= 1000;
                    }
                }
            }

            if (score < 0.0) {
                score = 0.0;
            }
    
       speed.erase(speed.begin());
        
    }
	org->score = score;
	org->dataMap.Append("score", score);
}

//print grid
void LoIWorld::printMap(int speed){
    for(int i=0;i<field.size();++i)
    {
        cout<<field[i];
    }
    cout<<endl;
    cout<<"speed"<<speed<<endl;
}

int LoIWorld::requiredInputs() {
	return numberOfInputs;
}
int LoIWorld::requiredOutputs() {
	return 1;
}
int LoIWorld::maxOrgsAllowed() {
	return 1;
}
int LoIWorld::minOrgsAllowed() {
	return 1;
}

