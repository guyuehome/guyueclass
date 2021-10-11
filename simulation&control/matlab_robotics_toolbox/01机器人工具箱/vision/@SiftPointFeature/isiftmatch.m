function m = isiftmatch(kp1, kp2)

    [matches,d] = siftmatch(kp1.d, kp2.d);

    m.correspond = matches';
    m.strength = d';
