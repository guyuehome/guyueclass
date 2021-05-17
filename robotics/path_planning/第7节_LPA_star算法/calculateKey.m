function key = calculateKey(Node,goalPos, rows, cols)
h = calculateH(Node.node, goalPos, rows, cols);
k1 = min(Node.g, Node.rhs) + h;
k2 = min(Node.g, Node.rhs);
key = [k1, k2];