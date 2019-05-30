#include "avi.h"
#include "generator.h"
#include <random>

void AVI::approximation(ValueFunction *valueFunction)
{
    //定义计数器，包括总模拟次数和每个instance的模拟次数
    int totalSimulationCount = 0;
    int lagApproximateCount = 0;
    bool startApproximate = false;
    while (totalSimulationCount++ < MAX_SIMULATION)
    {
        lagApproximateCount++;
        if (lagApproximateCount > LAG_APPROXIMATE)
        {
            startApproximate = true;
        }
        
        string fileName = "TrainingData.txt";
        Generator::instanceGenenrator(fileName);
        //初始化马尔科夫决策过程
        MDP simulation = MDP(fileName);
        vector<pair<Eigen::Vector4d, double> > valueAtThisSimulation;
        //开始mdp模拟
        while (simulation.currentState.currentRoute != nullptr)
        {
            Action bestAction;
            double value;
            simulation.findBestAction(&bestAction, *valueFunction, &value);
            //状态转移
            simulation.transition(bestAction);
            //记录这次sample path的信息
            simulation.solution.calcAttribute();
            valueAtThisSimulation.push_back(make_pair(simulation.solution.attribute, value));
        }
        //对lookup table 进行更新
        simulation.solution.calcCost();
        cout << totalSimulationCount << " " << simulation.solution.cost << endl;
        valueFunction->updateValue(valueAtThisSimulation, startApproximate);
        for (auto iter = simulation.customers.begin(); iter != simulation.customers.end(); ++iter)
        {
            delete iter->second;
        }
    }
}
