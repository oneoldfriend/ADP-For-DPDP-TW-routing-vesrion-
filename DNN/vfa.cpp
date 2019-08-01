#include "vfa.h"
#include "mdp.h"
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

    this->exe = new Executor(this->net, ctx_dev, in_args, arg_grad_store,
                             grad_req_type, aux_states);
}

double ValueFunction::getValue(State S, Action a, bool actor)
{
    double inputData[INPUT_DATA_FIRST_D][INPUT_DATA_SECOND_D];
    S.calcAttribute(a, inputData);

    this->exe->Forward(false);
    this->exe->outputs[0];
}

void ValueFunction::updateNetwork(double valueAtThisSimulation[(int)MAX_WORK_TIME][INPUT_DATA_FIRST_D][INPUT_DATA_SECOND_D], vector<double> rewardPath, bool startApproximate)
{
    //input data get

    exe->Forward(true);
    // update the parameters
    exe->Backward();
    for (int i = 1; i < 5; ++i)
    {
        this->exe->arg_arrays[i] -= this->exe->grad_arrays[i] * STEP_SIZE;
    }
}
