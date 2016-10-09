//  MABE is a product of The Hintza Lab @ MSU
//     for general research information:
//         http://hintzelab.msu.edu/
//     for MABE documentation:
//         https://github.com/ahnt/BasicMarkovBrainTemplate/wiki
//
//  Copyright (c) 2015 Michigan State University. All rights reserved.
//     to view the full license, visit:
//          https://github.com/ahnt/BasicMarkovBrainTemplate/wiki/license

//  This file was auto-generated with MBuilder.py

#ifndef __AutoBuild__Modules__
#define __AutoBuild__Modules__
#include "World/LoIWorld/LoIWorld.h"
#include "Genome/CircularGenome/CircularGenome.h"
#include "Genome/MultiGenome/MultiGenome.h"
#include "Brain/MarkovBrain/MarkovBrain.h"
#include "Brain/HumanBrain/HumanBrain.h"
#include "Optimizer/GAOptimizer/GAOptimizer.h"
#include "Optimizer/TournamentOptimizer/TournamentOptimizer.h"
#include "Optimizer/Tournament2Optimizer/Tournament2Optimizer.h"

#include "Archivist/DefaultArchivist.h"
#include "Archivist/LODwAPArchivist/LODwAPArchivist.h"
#include "Archivist/SSwDArchivist/SSwDArchivist.h"


//create a world
shared_ptr<AbstractWorld> makeWorld(shared_ptr<ParametersTable> PT = Parameters::root){
  shared_ptr<AbstractWorld> newWorld;
  bool found = false;
  string worldType = (PT == nullptr) ? AbstractWorld::worldTypePL->lookup() : PT->lookupString("WORLD-worldType");
  if (worldType == "LoI") {
    newWorld = make_shared<LoIWorld>(PT);
    found = true;
    }
  if (!found){
    cout << "  ERROR! could not find WORLD-worldType \"" << worldType << "\".\n  Exiting." << endl;
    exit(1);
    }
  return newWorld;
}


//create an optimizer
shared_ptr<AbstractOptimizer> makeOptimizer(shared_ptr<ParametersTable> PT = Parameters::root){
  shared_ptr<AbstractOptimizer> newOptimizer;
  bool found = false;
  string optimizerType = (PT == nullptr) ? AbstractOptimizer::Optimizer_MethodStrPL->lookup() : PT->lookupString("OPTIMIZER-optimizer");
  if (optimizerType == "GA") {
    newOptimizer = make_shared<GAOptimizer>(PT);
    found = true;
    }
  if (optimizerType == "Tournament") {
    newOptimizer = make_shared<TournamentOptimizer>(PT);
    found = true;
    }
  if (optimizerType == "Tournament2") {
    newOptimizer = make_shared<Tournament2Optimizer>(PT);
    found = true;
    }
  if (!found){
    cout << "  ERROR! could not find OPTIMIZER-optimizer \"" << optimizerType << "\".\n  Exiting." << endl;
    exit(1);
    }
  return newOptimizer;
}


//create an archivist
shared_ptr<DefaultArchivist> makeArchivist(vector<string> aveFileColumns, shared_ptr<ParametersTable> PT = Parameters::root){
  shared_ptr<DefaultArchivist> newArchivist;
  bool found = false;
  string archivistType = (PT == nullptr) ? DefaultArchivist::Arch_outputMethodStrPL->lookup() : PT->lookupString("ARCHIVIST-outputMethod");
  if (archivistType == "Default") {
    newArchivist = make_shared<DefaultArchivist>(aveFileColumns, PT);
    found = true;
    }
  if (archivistType == "LODwAP") {
    newArchivist = make_shared<LODwAPArchivist>(aveFileColumns, PT);
    found = true;
    }
  if (archivistType == "SSwD") {
    newArchivist = make_shared<SSwDArchivist>(aveFileColumns, PT);
    found = true;
    }
  if (!found){
    cout << "  ERROR! could not find ARCHIVIST-outputMethod \"" << archivistType << "\".\n  Exiting." << endl;
    exit(1);
    }
  return newArchivist;
}


//create a template genome
shared_ptr<AbstractGenome> makeTemplateGenome(shared_ptr<ParametersTable> PT = nullptr){
  shared_ptr<AbstractGenome> newGenome;
  bool found = false;
  string genomeType = (PT == nullptr) ? AbstractGenome::genomeTypeStrPL->lookup() : PT->lookupString("GENOME-genomeType");
  if (genomeType == "Circular") {
    newGenome = CircularGenome_genomeFactory(PT);
    found = true;
    }
  if (genomeType == "Multi") {
    newGenome = MultiGenome_genomeFactory(PT);
    found = true;
    }
  if (found == false){
    cout << "  ERROR! could not find GENOME-genomeType \"" << genomeType << "\".\n  Exiting." << endl;
    exit(1);
    }
  return newGenome;
}


//create a template brain
shared_ptr<AbstractBrain> makeTemplateBrain(shared_ptr<AbstractWorld> world, shared_ptr<ParametersTable> PT = nullptr){
  shared_ptr<AbstractBrain> newBrain;
  bool found = false;
  string brainType = (PT == nullptr) ? AbstractBrain::brainTypeStrPL->lookup() : PT->lookupString("BRAIN-brainType");
  int hiddenNodes = (PT == nullptr) ? AbstractBrain::hiddenNodesPL->lookup() : PT->lookupInt("BRAIN-hiddenNodes");
  if (brainType == "Markov") {
    newBrain = MarkovBrain_brainFactory(world->requiredInputs(), world->requiredOutputs(), hiddenNodes,PT);
    found = true;
    }
  if (brainType == "Human") {
    newBrain = HumanBrain_brainFactory(world->requiredInputs(), world->requiredOutputs(), hiddenNodes,PT);
    found = true;
    }
  if (found == false){
    cout << "  ERROR! could not find BRAIN-brainType \"" << brainType << "\".\n  Exiting." << endl;
    exit(1);
    }
  return newBrain;
}


//configure Defaults and Documentation
void configureDefaultsAndDocumentation(){
  Parameters::root->setParameter("BRAIN-brainType", (string)"Markov");
  Parameters::root->setDocumentation("BRAIN-brainType", "brain to be used, [Markov, Human]");

  Parameters::root->setParameter("GENOME-genomeType", (string)"Circular");
  Parameters::root->setDocumentation("GENOME-genomeType", "genome to be used, [Circular, Multi]");

  Parameters::root->setParameter("ARCHIVIST-outputMethod", (string)"Default");
  Parameters::root->setDocumentation("ARCHIVIST-outputMethod", "output method, [Default, LODwAP, SSwD]");

  Parameters::root->setParameter("OPTIMIZER-optimizer", (string)"GA");
  Parameters::root->setDocumentation("OPTIMIZER-optimizer", "optimizer to be used, [GA, Tournament, Tournament2]");

  Parameters::root->setParameter("WORLD-worldType", (string)"LoI");
  Parameters::root->setDocumentation("WORLD-worldType","world to be used, [LoI]");
}


#endif /* __AutoBuild__Modules__ */
