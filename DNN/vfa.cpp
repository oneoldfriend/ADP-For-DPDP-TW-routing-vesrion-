#include "vfa.h"
#include <iomanip>

ValueFunction::ValueFunction(const vector<int> &layers)
{
    auto x = Symbol::Variable("X");
    auto label = Symbol::Variable("label");

    weights = vector<Symbol>(layers.size());
    biases = vector<Symbol>(layers.size());
    outputs = vector<Symbol>(layers.size());

    for (size_t i = 0; i < layers.size(); ++i)
    {
        weights[i] = Symbol::Variable("w" + to_string(i));
        biases[i] = Symbol::Variable("b" + to_string(i));
        Symbol fc = FullyConnected(
            i == 0 ? x : outputs[i - 1], // data
            weights[i],
            biases[i],
            layers[i]);
        outputs[i] = i == layers.size() - 1 ? fc : Activation(fc, ActivationActType::kRelu);
    }
    this->net = SoftmaxOutput(outputs.back(), label);

    Context ctx_dev(DeviceType::kCPU, 0);

    NDArray array_x(Shape(INPUT_DATA_FIRST_D, INPUT_DATA_SECOND_D), ctx_dev, false);
    NDArray array_y(Shape(1), ctx_dev, false);

    mx_float *aptr_x = new mx_float[INPUT_DATA_FIRST_D * INPUT_DATA_SECOND_D];
    mx_float *aptr_y = new mx_float[1];

    array_x.SyncCopyFromCPU(aptr_x, INPUT_DATA_FIRST_D * INPUT_DATA_SECOND_D);
    array_x.WaitToRead();
    array_y.SyncCopyFromCPU(aptr_y, 1);
    array_y.WaitToRead();

    NDArray array_w_1(Shape(FIRST_LAYER, INPUT_DATA_SECOND_D), ctx_dev, false);
    NDArray array_b_1(Shape(FIRST_LAYER), ctx_dev, false);
    NDArray array_w_2(Shape(SECOND_LAYER, FIRST_LAYER), ctx_dev, false);
    NDArray array_b_2(Shape(SECOND_LAYER), ctx_dev, false);

    array_w_1 = 0.5f;
    array_b_1 = 0.0f;
    array_w_2 = 0.5f;
    array_b_2 = 0.0f;

    // the grads
    NDArray array_w_1_g(Shape(FIRST_LAYER, INPUT_DATA_SECOND_D), ctx_dev, false);
    NDArray array_b_1_g(Shape(FIRST_LAYER), ctx_dev, false);
    NDArray array_w_2_g(Shape(SECOND_LAYER, FIRST_LAYER), ctx_dev, false);
    NDArray array_b_2_g(Shape(SECOND_LAYER), ctx_dev, false);

    // Bind the symolic network with the ndarray
    // all the input args
    std::vector<NDArray> in_args;
    in_args.push_back(array_x);
    in_args.push_back(array_w_1);
    in_args.push_back(array_b_1);
    in_args.push_back(array_w_2);
    in_args.push_back(array_b_2);
    in_args.push_back(array_y);
    // all the grads
    std::vector<NDArray> arg_grad_store;
    arg_grad_store.push_back(NDArray()); // we don't need the grad of the input
    arg_grad_store.push_back(array_w_1_g);
    arg_grad_store.push_back(array_b_1_g);
    arg_grad_store.push_back(array_w_2_g);
    arg_grad_store.push_back(array_b_2_g);
    arg_grad_store.push_back(
        NDArray()); // neither do we need the grad of the loss
    // how to handle the grad
    std::vector<OpReqType> grad_req_type;
    grad_req_type.push_back(kNullOp);
    grad_req_type.push_back(kWriteTo);
    grad_req_type.push_back(kWriteTo);
    grad_req_type.push_back(kWriteTo);
    grad_req_type.push_back(kWriteTo);
    grad_req_type.push_back(kNullOp);

    std::vector<NDArray> aux_states;
    
    exe = new Executor(this->net, ctx_dev, in_args, arg_grad_store,
                       grad_req_type, aux_states);
}

double ValueFunction::getValue(State S, Action a, bool actor)
{
    S.calcAttribute(a);
    if (MYOPIC)
    {
        return 0;
    }
    else
    {
        if (actor)
        {
            return this->actorWeights.transpose() * S.attributes;
        }
        else
        {
            return this->criticWeights.transpose() * S.attributes;
        }
    }
}

void ValueFunction::updateActor(pair<Eigen::Vector4d, double> infoAtCurrentState, State nextState, Action actionForNextState, Eigen::Vector4d score)
{
    nextState.executeAction(actionForNextState);
    nextState.calcAttribute(actionForNextState);
    double estimateValue = this->criticWeights.transpose() * infoAtCurrentState.first;
    double bootstrappingValue = infoAtCurrentState.second + LAMBDA * this->criticWeights.transpose() * nextState.attributes;
    double error = bootstrappingValue - estimateValue;
    this->actorWeights = this->actorWeights + STEP_SIZE * score * error;
    double gammaN = 1.0 + infoAtCurrentState.first.transpose() * this->matrixBeta * infoAtCurrentState.first;
    this->criticWeights = this->criticWeights + 1 / gammaN * this->matrixBeta * infoAtCurrentState.first * error;
    this->matrixBeta = this->matrixBeta - 1.0 / gammaN * (this->matrixBeta * infoAtCurrentState.first * infoAtCurrentState.first.transpose() * this->matrixBeta);
    nextState.undoAction(actionForNextState);
}

void ValueFunction::updateCritic(vector<pair<Eigen::Vector4d, double>> valueAtThisSimulation, bool startApproximate, vector<Eigen::Vector4d> scoreAtThisSimulation)
{
    double lastValue = 0;
    for (auto iter = valueAtThisSimulation.rbegin(); iter != valueAtThisSimulation.rend(); ++iter)
    {
        iter->second += lastValue;
        lastValue = double(LAMBDA) * iter->second;
    }
    auto scoreIter = scoreAtThisSimulation.begin();
    for (auto iter = valueAtThisSimulation.begin(); iter != valueAtThisSimulation.end(); ++iter, ++scoreIter)
    {
        double gammaN = 1.0 + iter->first.transpose() * this->matrixBeta * iter->first,
               error = this->criticWeights.transpose() * iter->first - iter->second;
        this->criticWeights = this->criticWeights - 1 / gammaN * this->matrixBeta * iter->first * error;
        this->matrixBeta = this->matrixBeta - 1.0 / gammaN * (this->matrixBeta * iter->first * iter->first.transpose() * this->matrixBeta);
    }
}
