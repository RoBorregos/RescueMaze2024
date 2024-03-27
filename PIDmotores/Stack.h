// template <typename T>
// class Stack {
// private:
//     struct Node {
//         T data;
//         Node* next;
//         Node(const T& value) : data(value), next(nullptr) {}
//     };

//     Node* topNode;

// public:
//     Stack() : topNode(nullptr) {}

//     ~Stack() {
//         while (!isEmpty()) {
//             pop();
//         }
//     }

//     void push(const T& value) {
//         Node* newNode = new Node(value);
//         newNode->next = topNode;
//         topNode = newNode;
//     }

//     void pop() {
//         if (isEmpty()) {
//             throw std::runtime_error("Stack is empty");
//         }
//         Node* temp = topNode;
//         topNode = topNode->next;
//         delete temp;
//     }

//     T& top() {
//         if (isEmpty()) {
//             throw std::runtime_error("Stack is empty");
//         }
//         return topNode->data;
//     }

//     bool isEmpty() const {
//         return topNode == nullptr;
//     }
// };
