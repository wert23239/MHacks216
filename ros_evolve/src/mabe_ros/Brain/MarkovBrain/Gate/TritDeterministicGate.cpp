//  MABE is a product of The Hintze Lab @ MSU
//     for general research information:
//         hintzelab.msu.edu
//     for MABE documentation:
//         github.com/ahnt/MABE/wiki
//
//  Copyright (c) 2015 Michigan State University. All rights reserved.
//     to view the full license, visit:
//         github.com/ahnt/MABE/wiki/License

#include "TritDeterministicGate.h"

shared_ptr<ParameterLink<string>> TritDeterministicGate::IO_RangesPL = Parameters::register_parameter("BRAIN_MARKOV_GATES_TRIT-IO_Ranges", (string)"1-4,1-4", "range of number of inputs and outputs (min inputs-max inputs,min outputs-max outputs)");

TritDeterministicGate::TritDeterministicGate(pair<vector<int>, vector<int>> addresses, vector<vector<int>> _table, int _ID, shared_ptr<ParametersTable> _PT) :
	AbstractGate(_PT) {
	ID = _ID;
	inputs = addresses.first;
	outputs = addresses.second;
	table = _table;
}

void TritDeterministicGate::update(vector<double> & nodes, vector<double> & nextNodes) {
	int input = vectorToTritToInt(nodes,inputs,true);  // converts the input values into an index
	for (size_t i = 0; i < outputs.size(); i++) {
		nextNodes[outputs[i]] += table[input][i];
	}
}

shared_ptr<AbstractGate> TritDeterministicGate::makeCopy(shared_ptr<ParametersTable> _PT)
{
	if (_PT == nullptr) {
		_PT = PT;
	}
	auto newGate = make_shared<TritDeterministicGate>(_PT);
	newGate->table = table;
	newGate->ID = ID;
	newGate->inputs = inputs;
	newGate->outputs = outputs;
	return newGate;
}
