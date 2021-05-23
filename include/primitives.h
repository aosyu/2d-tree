#pragma once

#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <valarray>
#include <vector>

class Point
{
public:
    Point(double x, double y)
        : m_x(x)
        , m_y(y)
    {
    }

    double x() const { return m_x; }
    double y() const { return m_y; }
    double distance(const Point & p) const
    {
        const double x = p.x() - m_x;
        const double y = p.y() - m_y;
        return std::sqrt(x * x + y * y);
    }

    bool operator<(const Point & p) const { return (m_x == p.x() && m_y < p.y()) || (m_x < p.x()); }
    bool operator>(const Point & p) const { return (m_x == p.x() && m_y > p.y()) || (m_x > p.x()); }
    bool operator<=(const Point & p) const { return !(*this > p); }
    bool operator>=(const Point & p) const { return !(*this < p); }
    bool operator==(const Point & p) const { return m_x == p.x() && m_y == p.y(); }
    bool operator!=(const Point & p) const { return !(*this == p); }

    friend std::ostream & operator<<(std::ostream & out, const Point & p)
    {
        out << '(' << p.x() << ", " << p.y() << ')';
        return out;
    }

private:
    double m_x = 0;
    double m_y = 0;
};

class Rect
{
public:
    Rect(const Point & left_bottom, const Point & right_top)
        : m_left_bottom(left_bottom)
        , m_right_top(right_top)
    {
    }

    Rect()
        : m_left_bottom({std::numeric_limits<double>::min(), std::numeric_limits<double>::min()})
        , m_right_top({std::numeric_limits<double>::max(), std::numeric_limits<double>::max()})
    {
    }

    double xmin() const { return m_left_bottom.x(); }
    double ymin() const { return m_left_bottom.y(); }
    double xmax() const { return m_right_top.x(); }
    double ymax() const { return m_right_top.y(); }

    std::pair<Point, Point> getPoints()
    {
        return {m_left_bottom, m_right_top};
    }

    double distance(const Point & p) const
    {
        const double x = std::max({xmin() - p.x(), p.x() - xmax(), 0.0});
        const double y = std::max({ymin() - p.y(), p.y() - ymax(), 0.0});
        return sqrt(x * x + y * y);
    }

    bool contains(const Point & p) const { return p.x() > xmin() && p.y() > ymin() && p.x() < xmax() && p.y() < ymax(); }
    bool intersects(const Rect & rect) const { return !(m_right_top < rect.m_left_bottom) && !(rect.m_right_top < m_left_bottom); }

private:
    Point m_left_bottom;
    Point m_right_top;
};

namespace rbtree {

class PointSet
{
public:
    class iterator
    {
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const Point *;
        using reference = const Point &;
        using iterator_category = std::forward_iterator_tag;

        iterator() = default;
        iterator(const iterator & other) = default;

        iterator(const std::set<Point> & m_set)
        {
            m_ans_ptr = std::make_shared<std::vector<Point>>();
            for (auto & x : m_set) {
                m_ans_ptr->push_back(x);
            }
        }

        reference operator*() { return (*m_ans_ptr)[m_it]; }
        pointer operator->() { return (&(*m_ans_ptr)[m_it]); }

        iterator & operator++()
        {
            m_it++;
            return *this;
        }
        iterator operator++(int)
        {
            auto tmp = *this;
            operator++();
            return tmp;
        }
        iterator & operator=(const iterator & other)
        {
            m_it = other.m_it;
            m_ans_ptr = other.m_ans_ptr;
            return *this;
        }

        bool operator==(const iterator & other) const { return *m_ans_ptr == *other.m_ans_ptr && m_it == other.m_it; }
        bool operator!=(const iterator & other) const { return !(*this == other); }

        void end()
        {
            m_it = m_ans_ptr->size();
        }

    private:
        int m_it = 0;

        std::shared_ptr<std::vector<Point>> m_ans_ptr;
    };

    PointSet(const std::string & filename = {})
    {
        std::ifstream in(filename);
        if (in.is_open()) {
            double x, y;
            while (in >> x >> y) {
                m_tree.emplace(x, y);
            }
        }
    }

    bool empty() const { return m_tree.empty(); }
    std::size_t size() const { return m_tree.size(); }
    void put(const Point & p) { m_tree.insert(p); }
    bool contains(const Point & p) const { return m_tree.end() != m_tree.find(p); }

    std::pair<iterator, iterator> range(const Rect & r) const
    {
        std::set<Point> m_range_result;
        for (const auto it : m_tree) {
            if (r.contains(it)) {
                m_range_result.insert(it);
            }
        }
        iterator begin(m_range_result);
        iterator end(m_range_result);
        end.end();
        return {begin, end};
    }

    iterator begin() const { return iterator(m_tree); }
    iterator end() const
    {
        iterator end = begin();
        end.end();
        return end;
    }

    std::optional<Point> nearest(const Point & p) const
    {
        return (*std::min_element(begin(), end(), [&p](const Point & a, const Point & b) {
            return a.distance(p) < b.distance(p);
        }));
    }

    std::pair<iterator, iterator> nearest(const Point & p, std::size_t k) const
    {
        std::set<Point> m_nearest_set;
        std::map<double, Point> map;
        for (const auto it : m_tree) {
            map.insert({p.distance(it), it});
        }

        auto it_ans = map.begin();
        for (std::size_t i = 0; i < k; i++) {
            if (it_ans == map.end()) {
                break;
            }
            m_nearest_set.emplace(it_ans->second);
            it_ans++;
        }
        iterator begin(m_nearest_set);
        iterator end(m_nearest_set);
        end.end();
        return {begin, end};
    }

    friend std::ostream & operator<<(std::ostream & out, const PointSet & ps)
    {
        for (const auto & it : ps) {
            out << it;
        }
        return out;
    }

private:
    std::set<Point> m_tree;
};

} // namespace rbtree

namespace kdtree {

class PointSet
{
public:
    struct Node
    {
        Node(const Point & p, bool v, const Rect & r)
            : vertical(v)
            , point(p)
            , rect(r)
        {
        }
        bool vertical = true;
        const Point point;
        const Rect rect;
        std::shared_ptr<Node> left;
        std::shared_ptr<Node> right;
    };

    class iterator
    {
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const value_type *;
        using reference = const value_type &;
        using iterator_category = std::forward_iterator_tag;

        iterator() = default;
        iterator(const PointSet & p, std::size_t i)
            : m_it(i)
        {
            m_ans_ptr = std::make_shared<std::vector<Point>>();
            init(p.m_root);
        }

        void init(const std::shared_ptr<Node> & node)
        {
            if (node == nullptr) {
                return;
            }
            m_ans_ptr->push_back(node->point);
            init(node->left);
            init(node->right);
        }

        reference operator*() { return (*m_ans_ptr)[m_it]; }
        pointer operator->() { return &operator*(); }
        iterator & operator++()
        {
            m_it++;
            return *this;
        }
        iterator operator++(int)
        {
            auto tmp = *this;
            operator++();
            return tmp;
        }
        bool operator==(const iterator & other) const
        {
            if (m_it != other.m_it || m_ans_ptr->size() != other.m_ans_ptr->size()) {
                return false;
            }
            for (size_t i = 0; i < m_ans_ptr->size(); i++) {
                if ((*m_ans_ptr)[i] != (*other.m_ans_ptr)[i]) {
                    return false;
                }
            }
            return true;
        }
        bool operator!=(const iterator & other) const { return !(operator==(other)); }

        void end()
        {
            m_it = m_ans_ptr->size();
        }
    private:
        int m_it = 0;

        std::shared_ptr<std::vector<Point>> m_ans_ptr;
    };

    PointSet(const std::string & filename = {});

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    std::pair<iterator, iterator> range(const Rect &) const;
    iterator begin() const;
    iterator end() const;

    std::optional<Point> nearest(const Point &) const;
    std::pair<iterator, iterator> nearest(const Point &, std::size_t) const;

    friend std::ostream & operator<<(std::ostream &, const PointSet &);

private:
    static std::function<bool(Point, Point)> pointComparator(const Point & p)
    {
        return [&p](Point a, Point b) { return p.distance(a) < p.distance(b); };
    }
    std::shared_ptr<Node> m_root = nullptr;
    std::size_t m_size = 0;

    bool contains(const std::shared_ptr<Node> & current, const Point & p) const;
    void put(std::shared_ptr<Node> & current, const Point & p, bool isLeftChild, const std::shared_ptr<Node> & parent);
    void range(const Rect & r, const std::shared_ptr<Node> & node, PointSet & m_range_result) const;
    void nearest(const Point & p, const Node & current, std::set<Point, decltype(pointComparator(p))> & m_nearest_answer, size_t k) const;
    bool needToGoLeft(const std::shared_ptr<Node> & current, const Point & p) const;
    Rect updateCoordinates(const std::shared_ptr<Node>& parent, bool isLeftChild);
    std::shared_ptr<Node> makeTree(std::vector<Point> & input, bool vertical, Rect coordinates);
    std::shared_ptr<Node> getChildPtr(const std::shared_ptr<Node> & parent, bool isLeftChild, const Point & p);
};

} // namespace kdtree
