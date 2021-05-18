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
        sort(input.begin(), input.end(), [](const Point & first, const Point & second) {
            return first.x() < second.x();
        });
        m_size = 1;
        Point med = input[input.size() / 2];
        m_root = std::make_shared<Node>(med, true, Rect({INT32_MIN, INT32_MIN}, {INT32_MAX, INT32_MAX}));
        makeTree(input, m_root);
    }
    in.close();
}

void sortByX(std::vector<Point> & input)
{
    sort(input.begin(), input.end(), [](const Point & first, const Point & second) {
        return first.x() < second.x();
    });
}
void sortByY(std::vector<Point> & input)
{
    sort(input.begin(), input.end(), [](const Point & first, const Point & second) {
        return first.y() < second.y();
    });
}

void PointSet::updateCoordinates(const nodePtr & parent, bool isLeftChild, double & xmin, double & xmax, double & ymin, double & ymax)
{
    if (isLeftChild) {
        if (parent->vertical) {
            xmax = parent->point.x();
        }
        else {
            ymax = parent->point.y();
        }
    }
    else {
        if (parent->vertical) {
            xmin = parent->point.x();
        }
        else {
            ymin = parent->point.y();
        }
    }
}

std::shared_ptr<Node> PointSet::getChildPtr(const nodePtr & parent, bool isLeftChild, const Point & p)
{
    double xmin = parent->rect.xmin(), xmax = parent->rect.xmax(), ymin = parent->rect.ymin(), ymax = parent->rect.ymax();
    updateCoordinates(parent, isLeftChild, xmin, xmax, ymin, ymax);
    return std::make_shared<Node>(p, !parent->vertical, Rect({xmin, ymin}, {xmax, ymax}));
}

void PointSet::makeTree(std::vector<Point> & input, const nodePtr & current)
{
    if (input.size() == 1) {
        return;
    }
    m_size++;
    std::size_t const half_size = input.size() / 2;
    std::vector<Point> left(input.begin(), input.begin() + half_size);
    std::vector<Point> right(input.begin() + half_size, input.end());
    if (current->vertical) {
        sortByX(left);
        sortByX(right);
    }
    else {
        sortByY(left);
        sortByY(right);
    }

    current->left = getChildPtr(current, true, left[left.size() / 2]);
    makeTree(left, current->left);
    current->right = getChildPtr(current, false, right[right.size() / 2]);
    makeTree(right, current->right);
}

bool PointSet::empty() const
{
    return m_size == 0;
}
std::size_t PointSet::size() const
{
    return m_size;
}

void PointSet::put(const Point & p)
{
    if (m_root == nullptr) {
        m_size = 1;
        m_root = std::make_shared<Node>(p, true, Rect({INT32_MIN, INT32_MIN}, {INT32_MAX, INT32_MAX}));
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
    else if (current->point == p) {
        return true;
    }
    else if (needToGoLeft(current, p)) {
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
        ((!current.vertical && p.x() <= current.point.x()) ||
         (current.vertical && p.y() <= current.point.y()) || current.right == nullptr)) {
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
    for (const auto i : m_nearest_answer) {
        ans.put(i);
    }
    return {ans.begin(), ans.end()};
}

std::ostream & operator<<(std::ostream & out, const PointSet & p)
{
    for (const auto it : p) {
        out << &it << std::endl;
    }
    return out;
}
} // namespace kdtree