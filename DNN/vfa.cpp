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

    Symbol null_sym;
    for (int i = 0; i < layers.size(); i++)
    {
        std::string istr = std::to_string(i);
        weights[i] = Symbol::Variable(std::string("w") + istr);
        biases[i] = Symbol::Variable(std::string("b") + istr);
        Symbol fc = FullyConnected(std::string("fc") + istr,
                                   i == 0 ? x : outputs[i - 1],
                                   weights[i], biases[i], layers[i]);
        outputs[i] = LeakyReLU(std::string("act") + istr, fc, null_sym, LeakyReLUActType::kLeaky);
    }
    this->net = SoftmaxOutput(outputs.back(), label);

    Context ctx_dev(DeviceType::kCPU, 0);

    NDArray array_x(Shape(1, INPUT_LAYER), ctx_dev, false);
    NDArray array_y(Shape(1), ctx_dev, false);

    NDArray array_w_1(Shape(HIDDEN_LAYER, INPUT_LAYER), ctx_dev, false);
    NDArray array_b_1(Shape(HIDDEN_LAYER), ctx_dev, false);
    NDArray array_w_2(Shape(1, HIDDEN_LAYER), ctx_dev, false);
    NDArray array_b_2(Shape(1), ctx_dev, false);

    array_w_1 = 0.5f;
    array_b_1 = 0.0f;
    array_w_2 = 0.5f;
    array_b_2 = 0.0f;

    // the grads

    NDArray array_w_1_g(Shape(HIDDEN_LAYER, INPUT_LAYER), ctx_dev, false);
    NDArray array_b_1_g(Shape(HIDDEN_LAYER), ctx_dev, false);
    NDArray array_w_2_g(Shape(1, HIDDEN_LAYER), ctx_dev, false);
    NDArray array_b_2_g(Shape(1), ctx_dev, false);

    // Bind the symbolic network with the ndarray
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
    if (MYOPIC)
    {
        return 0;
    }
    else
    {
        double inputData[INPUT_DATA_FIRST_D][INPUT_DATA_SECOND_D];
        S.calcAttribute(a, inputData);
        mx_float *aptr_x = new mx_float[INPUT_LAYER];
        for (int i = 0; i < INPUT_DATA_FIRST_D; i++)
        {
            for (int j = 0; j < INPUT_DATA_SECOND_D; j++)
            {
                aptr_x[i * INPUT_DATA_SECOND_D + j] = inputData[i][j];
            }
        }
        this->exe->arg_arrays[0].SyncCopyFromCPU(aptr_x, INPUT_LAYER);
        this->exe->arg_arrays[0].WaitToRead();
        this->exe->Forward(false);
        this->exe->outputs[0];
        delete[] aptr_x;
    }
}

void ValueFunction::updateNetwork(double valueAtThisSimulation[(int)MAX_WORK_TIME][INPUT_DATA_FIRST_D][INPUT_DATA_SECOND_D], vector<double> rewardPath, bool startApproximate)
{
    double lastValue = 0.0;
    for (auto iter = rewardPath.rbegin(); iter != rewardPath.rend(); ++iter)
    {
        *iter += lastValue;
        lastValue = double(LAMBDA) * *iter;
    }

    for (int simulation = 0; simulation < rewardPath.size(); simulation++)
    {
        mx_float *aptr_x = new mx_float[INPUT_LAYER];
        mx_float *aptr_y = new mx_float[1];
        for (int i = 0; i < INPUT_DATA_FIRST_D; i++)
        {
            for (int j = 0; j < INPUT_DATA_SECOND_D; j++)
            {
                aptr_x[i * INPUT_DATA_SECOND_D + j] = valueAtThisSimulation[simulation][i][j];
            }
        }
        aptr_y[0] = rewardPath[simulation];
        this->exe->arg_arrays[0].SyncCopyFromCPU(aptr_x, INPUT_LAYER);
        this->exe->arg_arrays[0].WaitToRead();
        this->exe->arg_arrays[5].SyncCopyFromCPU(aptr_y, 1);
        this->exe->arg_arrays[5].WaitToRead();
        exe->Forward(true);
        // update the parameters
        exe->Backward();
        for (int i = 1; i < 5; ++i)
        {
            this->exe->arg_arrays[i] -= this->exe->grad_arrays[i] * STEP_SIZE;
        }
        delete[] aptr_x;
        delete[] aptr_y;
    }
}
