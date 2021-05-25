#include "primitives.h"

namespace kdtree {

using Node = PointSet::Node;
using iterator = PointSet::iterator;
using nodePtr = std::shared_ptr<Node>;

PointSet::PointSet(const std::string & filename)
{
    std::ifstream in(filename);
    m_tree = std::make_shared<std::vector<Point>>();
    std::vector<Point> input;
    if (in.is_open()) {
        double x, y;
        while (in >> x >> y) {
            input.push_back({x, y});
        }
    }
    if (!input.empty()) {
        m_root = makeTree(input, true, {});
    }
    traverse(m_root);
}

Rect PointSet::updateCoordinates(const std::shared_ptr<Node> & parent, bool isLeftChild)
{
    Rect rect = parent->rect;
    auto [bot, top] = rect.getPoints();
    Point point = parent->point;

    if (isLeftChild) {
        if (parent->vertical) {
            return {bot, {point.x(), top.y()}};
        }
        return {bot, {top.x(), point.y()}};
    }

    if (parent->vertical) {
        return {{point.x(), bot.y()}, top};
    }

    return {{bot.x(), point.y()}, top};
}

std::shared_ptr<Node> PointSet::getChildPtr(const nodePtr & parent, bool isLeftChild, const Point & p)
{
    Rect newRect = updateCoordinates(parent, isLeftChild);
    return std::make_shared<Node>(p, !parent->vertical, newRect);
}

std::shared_ptr<Node> PointSet::makeTree(std::vector<Point> & input, bool vertical, Rect coordinates)
{
    // по какой-то причине Run make требовал сделать input const, при чем сортировка невозможна, поэтому я вынуждена поставить костыль
    input.push_back({0, 0});
    input.pop_back();
    // конец костыля

    if (input.size() == 1) {
        return std::make_shared<Node>(input[0], vertical, coordinates);
    }

    if (vertical) {
        sort(input.begin(), input.end(), [](const Point & first, const Point & second) {
            return first.x() < second.x();
        });
    }
    else {
        sort(input.begin(), input.end(), [](const Point & first, const Point & second) {
            return first.y() < second.y();
        });
    }

    std::size_t const half_size = input.size() / 2;
    std::vector<Point> left(input.begin(), input.begin() + half_size);
    std::vector<Point> right(input.begin() + half_size, input.end());

    std::shared_ptr<Node> self = std::make_shared<Node>(input[input.size() / 2], vertical, coordinates);

    self->left = makeTree(left, !vertical, updateCoordinates(self, true));
    self->right = makeTree(right, !vertical, updateCoordinates(self, false));

    return self;
}

bool PointSet::empty() const
{
    update_m_tree_if_needed();
    return m_tree->empty();
}

std::size_t PointSet::size() const
{
    update_m_tree_if_needed();
    return m_tree->size();
}

void PointSet::put(const Point & p)
{
    if (m_root == nullptr) {
        m_root = std::make_shared<Node>(p, true, Rect());
        m_tree->push_back(m_root->point);
        return;
    }
    if (!contains(p)) {
        put(m_root, p, false, m_root);
    }
}

bool PointSet::needToGoLeft(const nodePtr & current, const Point & p) const
{
    return ((!current->vertical || current == m_root) && p.x() <= current->point.x()) ||
            ((current->vertical || current == m_root) && p.y() <= current->point.y());
}

void PointSet::put(nodePtr & current, const Point & p, bool isLeftChild, const nodePtr & parent)
{
    if (current == nullptr) {
        current = getChildPtr(parent, isLeftChild, p);
        unhandled_put = true;
    }
    else if (needToGoLeft(current, p)) {
        put(current->left, p, true, current);
    }
    else {
        put(current->right, p, false, current);
    }
}

bool PointSet::contains(const nodePtr & current, const Point & p) const
{
    if (current == nullptr) {
        return false;
    }
    if (current->point == p) {
        return true;
    }
    if (needToGoLeft(current, p)) {
        return contains(current->left, p);
    }
    return contains(current->right, p);
}
bool PointSet::contains(const Point & p) const { return !empty() && contains(m_root, p); }

void PointSet::traverse(const std::shared_ptr<Node> & node) const
{
    if (node == nullptr) {
        return;
    }
    m_tree->push_back(node->point);
    traverse(node->left);
    traverse(node->right);
}

void PointSet::update_m_tree_if_needed() const
{
    if (unhandled_put) {
        m_tree->clear();
        traverse(m_root);
        unhandled_put = false;
    }
}

PointSet::iterator PointSet::begin() const
{
    update_m_tree_if_needed();
    return iterator(m_tree, m_tree->begin());
}

PointSet::iterator PointSet::end() const
{
    update_m_tree_if_needed();
    return iterator(m_tree, m_tree->end());
}

void PointSet::range(const Rect & r, const nodePtr & node, const std::shared_ptr<std::vector<Point>> & result) const
{
    if (node == nullptr || !r.intersects(node->rect)) {
        return;
    }
    if (r.contains(node->point)) {
        result->push_back(node->point);
    }
    range(r, node->left, result);
    range(r, node->right, result);
}

std::pair<iterator, iterator> PointSet::range(const Rect & r) const
{
    std::shared_ptr<std::vector<Point>> ans_ptr = std::make_shared<std::vector<Point>>();
    range(r, m_root, ans_ptr);
    iterator begin(ans_ptr, ans_ptr->begin());
    iterator end(ans_ptr, ans_ptr->end());
    return {begin, end};
}

void PointSet::nearest(const Point & p, const Node & current, std::set<Point, decltype(pointComparator(p))> & result, const size_t k) const
{
    if (p.distance(*prev(result.end())) >= p.distance(current.point) || result.size() < k) {
        if (result.size() == k) {
            result.erase(prev(result.end()));
        }
        result.insert(current.point);
    }

    if (current.left != nullptr &&
        ((!current.vertical && p.x() < current.point.x()) ||
         (current.vertical && p.y() < current.point.y()) || current.right == nullptr)) {

        nearest(p, *current.left, result, k);

        if (current.right != nullptr && (p.distance(*prev(result.end())) >= current.right->rect.distance(p) || result.size() < k)) {
            nearest(p, *current.right, result, k);
        }
    }
    else if (current.right != nullptr) {
        nearest(p, *current.right, result, k);

        if (current.left != nullptr && (p.distance(*prev(result.end())) >= current.left->rect.distance(p) || result.size() < k)) {
            nearest(p, *current.left, result, k);
        }
    }
}

std::optional<Point> PointSet::nearest(const Point & p) const
{
    std::set<Point, decltype(pointComparator(p))> answer(pointComparator(p));
    answer.insert(m_root->point);
    nearest(p, *m_root, answer, 1);
    return *answer.begin();
}

std::pair<iterator, iterator> PointSet::nearest(const Point & p, std::size_t k) const
{
    if (k == 0) {
        return {{}, {}};
    }

    std::set<Point, decltype(pointComparator(p))> answer(pointComparator(p));
    answer.insert(m_root->point);
    nearest(p, *m_root, answer, k);

    std::shared_ptr<std::vector<Point>> ans_ptr = std::make_shared<std::vector<Point>>(answer.begin(), answer.end());
    iterator begin(ans_ptr, ans_ptr->begin());
    iterator end(ans_ptr, ans_ptr->end());
    return {begin, end};
}

std::ostream & operator<<(std::ostream & out, const PointSet & p)
{
    for (const auto it : p) {
        out << &it << "\n";
    }
    return out;
}

} // namespace kdtree