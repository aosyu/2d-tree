#include "primitives.h"

namespace kdtree {

using Node = PointSet::Node;
using iterator = PointSet::iterator;
using nodePtr = std::shared_ptr<Node>;

PointSet::PointSet(const std::string & filename)
{
    std::ifstream in(filename);
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
}

Rect PointSet::updateCoordinates(const std::shared_ptr<Node>& parent, bool isLeftChild)
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
    if (input.empty()) {
        return nullptr;
    }

    m_size++;
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

bool PointSet::empty() const { return m_size == 0; }

std::size_t PointSet::size() const { return m_size; }

void PointSet::put(const Point & p)
{
    if (m_root == nullptr) {
        m_size = 1;
        m_root = std::make_shared<Node>(p, true, Rect());
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
        m_size++;
        current = getChildPtr(parent, isLeftChild, p);
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

PointSet::iterator PointSet::begin() const { return iterator(*this, 0); }
PointSet::iterator PointSet::end() const { return iterator(*this, size()); }

void PointSet::range(const Rect & r, const nodePtr & node, PointSet & m_range_result) const
{
    if (node == nullptr || !r.intersects(node->rect)) {
        return;
    }
    if (r.contains(node->point)) {
        m_range_result.put(node->point);
    }
    range(r, node->left, m_range_result);
    range(r, node->right, m_range_result);
}

std::pair<iterator, iterator> PointSet::range(const Rect & r) const
{
    PointSet m_range_result;
    range(r, m_root, m_range_result);
    return {m_range_result.begin(), m_range_result.end()};
}

void PointSet::nearest(const Point & p, const Node & current, std::set<Point, decltype(pointComparator(p))> & m_nearest_answer, const size_t k) const
{
    if (p.distance(*prev(m_nearest_answer.end())) >= p.distance(current.point) || m_nearest_answer.size() < k) {
        if (m_nearest_answer.size() == k) {
            m_nearest_answer.erase(prev(m_nearest_answer.end()));
        }
        m_nearest_answer.insert(current.point);
    }

    if (current.left != nullptr &&
        ((!current.vertical && p.x() < current.point.x()) ||
         (current.vertical && p.y() < current.point.y()) || current.right == nullptr)) {

        nearest(p, *current.left, m_nearest_answer, k);

        if (current.right != nullptr && (p.distance(*prev(m_nearest_answer.end())) >= current.right->rect.distance(p) || m_nearest_answer.size() < k)) {
            nearest(p, *current.right, m_nearest_answer, k);
        }
    }
    else if (current.right != nullptr) {
        nearest(p, *current.right, m_nearest_answer, k);

        if (current.left != nullptr && (p.distance(*prev(m_nearest_answer.end())) >= current.left->rect.distance(p) || m_nearest_answer.size() < k)) {
            nearest(p, *current.left, m_nearest_answer, k);
        }
    }
}

std::optional<Point> PointSet::nearest(const Point & p) const
{
    std::set<Point, decltype(pointComparator(p))> m_nearest_answer(pointComparator(p));
    m_nearest_answer.insert(m_root->point);
    nearest(p, *m_root, m_nearest_answer, 1);
    return *m_nearest_answer.begin();
}
std::pair<iterator, iterator> PointSet::nearest(const Point & p, std::size_t k) const
{
    PointSet ans;
    if (k == 0) {
        return {ans.begin(), ans.end()};
    }
    std::set<Point, decltype(pointComparator(p))> m_nearest_answer(pointComparator(p));
    m_nearest_answer.insert(m_root->point);
    nearest(p, *m_root, m_nearest_answer, k);
    for (const auto & i : m_nearest_answer) {
        ans.put(i);
    }
    return {ans.begin(), ans.end()};
}

std::ostream & operator<<(std::ostream & out, const PointSet & p)
{
    for (const auto it : p) {
        out << &it << "\n";
    }
    return out;
}
} // namespace kdtree