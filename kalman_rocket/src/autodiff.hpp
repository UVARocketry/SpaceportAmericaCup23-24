#include <cmath>
#include <memory>
#include <optional>
#include <stack>
#include <unordered_set>
#include <vector>

// This is a reverse topological sort written by chatgpt for the autodiff
template <class T>
class ReverseTopologicalSort {
public:
    static std::vector<std::shared_ptr<T>>
    reverseTopoSort(const std::shared_ptr<T>& startNode) {
        std::vector<std::shared_ptr<T>> result;
        std::unordered_set<std::shared_ptr<T>> visited;
        std::stack<std::shared_ptr<T>> stack;

        reverseTopoSortUtil(startNode, visited, stack);

        while (!stack.empty()) {
            result.push_back(stack.top());
            stack.pop();
        }

        return result;
    }

private:
    static void reverseTopoSortUtil(
        const std::shared_ptr<T>& node,
        std::unordered_set<std::shared_ptr<T>>& visited,
        std::stack<std::shared_ptr<T>>& stack
    ) {
        visited.insert(node);

        for (const auto& parent : node->inputs) {
            if (visited.find(parent) == visited.end()) {
                reverseTopoSortUtil(parent, visited, stack);
            }
        }

        stack.push(node);
    }
};

template <typename T>
struct AutoDiffPtr;
// Class for backwards autodiff
template <typename T>
struct BackwardAutoDiff {
    typedef BackwardAutoDiff<T> Self;
    enum OpTp {
        Const,
        Add,
        Cos,
        Sin,
        Sub,
        Mul,
        Div,
    };
    T constInput;
    std::vector<std::shared_ptr<BackwardAutoDiff<T>>> inputs;
    std::optional<T> output;
    const OpTp op;
    std::optional<T> gradient;
    BackwardAutoDiff(OpTp op) : op(op) {
    }
    BackwardAutoDiff(T val) : op(Const), constInput(val) {
    }
    static AutoDiffPtr<T> makeConst(T val) {
        auto ret = std::make_shared<Self>(Const);
        ret->constInput = val;
        return ret;
    }

    T get_gradient() {
        if (!this->gradient.has_value()) {
            return 0;
        }
        return this->gradient.value();
    }
    void reset() {
        this->output.reset();
        this->gradient.reset();
        for (auto& input : this->inputs) {
            input->reset();
        }
    }

    void forward() {
        if (this->output.has_value()) {
            return;
        }
        switch (this->op) {
        case Const:
            this->output = this->constInput;
            break;
        case Add:
            this->inputs[0]->forward();
            this->inputs[1]->forward();
            this->output = this->inputs[0]->output.value() +
                           this->inputs[1]->output.value();
            break;
        case Sub:
            this->inputs[0]->forward();
            this->inputs[1]->forward();
            this->output = this->inputs[0]->output.value() -
                           this->inputs[1]->output.value();
            break;
        case Mul:
            this->inputs[0]->forward();
            this->inputs[1]->forward();
            this->output = this->inputs[0]->output.value() *
                           this->inputs[1]->output.value();
            break;
        case Cos:
            this->inputs[0]->forward();
            this->output = cos(this->inputs[0]->output.value());
            break;
        case Sin:
            this->inputs[0]->forward();
            this->output = sin(this->inputs[0]->output.value());
            break;
        case Div:
            this->inputs[0]->forward();
            this->inputs[1]->forward();
            this->output = this->inputs[0]->output.value() /
                           this->inputs[1]->output.value();

            break;
        }
    }
    static void backward(std::shared_ptr<Self> node) {
        auto reversed = ReverseTopologicalSort<Self>::reverseTopoSort(node);
        for (std::shared_ptr<Self>& node : reversed) {
            for (std::shared_ptr<Self>& input : node->inputs) {
                if (!input->gradient.has_value()) {
                    input->gradient = 0;
                }
                input->gradient.value();
            }
            if (!node->gradient.has_value()) {
                node->gradient = 0;
            }
            switch (node->op) {
            case Const:
                break;
            case Add:
                node->inputs[0]->gradient.value() += node->gradient.value();
                node->inputs[1]->gradient.value() += node->gradient.value();
                break;
            case Sub:
                node->inputs[0]->gradient.value() += node->gradient.value();
                node->inputs[1]->gradient.value() -= node->gradient.value();
                break;
            case Mul:
                node->inputs[0]->gradient.value() +=
                    node->gradient.value() * node->inputs[1]->output.value();
                node->inputs[1]->gradient.value() +=
                    node->gradient.value() * node->inputs[0]->output.value();
                break;
            case Div:
                node->inputs[0]->gradient.value +=
                    node->gradient.value() / node->inputs[1]->output.value();
                node->inputs[1]->gradient.value() +=
                    -node->gradient.value() * node->inputs[0]->output.value();

            case Cos:
                node->inputs[0]->gradient.value() +=
                    node->gradient.value() *
                    -sin(node->inputs[0]->output.value());
                break;
            case Sin:
                node->inputs[0]->gradient.value() +=
                    node->gradient.value() *
                    cos(node->inputs[0]->output.value());
                break;
            }
        }
    }
};

template <typename T>
struct AutoDiffPtr {
    std::shared_ptr<BackwardAutoDiff<T>> ptr;
    typedef AutoDiffPtr<T> Self;
    AutoDiffPtr(std::shared_ptr<BackwardAutoDiff<T>> ptr) : ptr(ptr) {
    }
    AutoDiffPtr(T val) : ptr(BackwardAutoDiff<T>::makeConst(val)) {
    }
    T get_gradient() {
        return this->ptr->get_gradient();
    }
    void reset() {
        this->ptr->reset();
    }
    Self operator+(Self other) {
        auto ret =
            std::make_shared<BackwardAutoDiff<T>>(BackwardAutoDiff<T>::OpTp::Add
            );
        ret->inputs.push_back(this->ptr);
        ret->inputs.push_back(other.ptr);
        return ret;
    }
    Self operator*(Self other) {
        auto ret =
            std::make_shared<BackwardAutoDiff<T>>(BackwardAutoDiff<T>::OpTp::Mul
            );
        ret->inputs.push_back(this->ptr);
        ret->inputs.push_back(other.ptr);
        return ret;
    }
    operator std::shared_ptr<BackwardAutoDiff<T>>() {
        return this->ptr;
    }
    friend Self cos(Self a) {
        auto ret =
            std::make_shared<BackwardAutoDiff<T>>(BackwardAutoDiff<T>::OpTp::Cos
            );
        ret->inputs.push_back(a);
        return ret;
    }
    friend Self sin(Self a) {
        auto ret =
            std::make_shared<BackwardAutoDiff<T>>(BackwardAutoDiff<T>::OpTp::Sin
            );
        ret->inputs.push_back(a);
        return ret;
    }
    BackwardAutoDiff<T>& operator*() {
        return *this->ptr;
    }
    BackwardAutoDiff<T>* operator->() {
        return this->ptr.get();
    }
};
