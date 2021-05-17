function key = calculateKey(Node,s_start, km, rows, cols)
h = calculateH(Node.node, s_start, rows, cols);
k1 = min(Node.g, Node.rhs) + h + km;
k2 = min(Node.g, Node.rhs);
key = [k1, k2];