def find_sequence(number_of_nodes: int, *, minimum_distance: int) -> list[int]:
    """Find a sequence of nodes that fulfills the following constraints.

    Constraints:
    - the sequence must contain all nodes from 0 to number_of_nodes - 1
    - the sequence starts with node 0
    - the distance between two nodes must be at least min_distance
    - the distance between two nodes must be less than 2 * min_distance
    - the sequence increases and decreases alternately
    """
    if number_of_nodes < 2 * minimum_distance + 1:
        return []
    nodes = list(range(number_of_nodes))
    sequence = [nodes.pop(0)]
    while len(nodes) > 2 * minimum_distance + 1:
        back = nodes.pop(0)
        index = 0
        while nodes[index] - back < minimum_distance:
            index += 1
        sequence.append(nodes.pop(index))
        sequence.append(back)
    for i in range(minimum_distance):
        sequence.append(nodes[minimum_distance + i])
        sequence.append(nodes[i])
    for _ in range(minimum_distance):
        nodes.pop(0)
        nodes.pop(0)
    if nodes:
        sequence.append(nodes.pop())
    return sequence
