#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <iostream>
#include <vector>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// よく使う型を定義
// 点：座標値は float 型, 2次元,　直交座標系
using point = bg::model::point<float, 2, bg::cs::cartesian>;
// 正方形：端点が point 型
using box = bg::model::box<point>;
// uint16_t の属性を持った box
using value = std::pair<box, uint16_t>;

int main() {
  // uint16_t の属性を持つ box　に対する rtree. ノードの最大数は 16.
  bgi::rtree<value, bgi::quadratic<16> > rtree;

  // サンプルデータ作成
  for (uint16_t index = 0; index < 10; ++index) {
    auto target = static_cast<float>(index);
    // box 型のサンプルデータ
    box b(point(target + 0.0f, target + 0.0f),
          point(target + 0.5f, target + 0.5f));
    // R-tree に登録
    rtree.insert(std::make_pair(b, index));
  }

  // 検索その1：query_box と交差する box をすべて取得
  {
    // query となる box を定義
    box query_box(point(0, 0), point(5, 5));
    // Spatial query の結果
    std::vector<value> result_s;
    // https://www.boost.org/doc/libs/1_78_0/libs/geometry/doc/html/geometry/spatial_indexes/queries.html
    rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));

    // Spatial query の結果を表示
    // bg::wkt は WKT (Well-Known Text) 形式で幾何形状を表示
    // https://www.boost.org/doc/libs/1_65_0/libs/geometry/doc/html/geometry/reference/io/wkt.html
    // https://ja.wikipedia.org/wiki/Well-known_text
    std::cout << "spatial query box:" << std::endl;
    std::cout << bg::wkt<box>(query_box) << std::endl;
    std::cout << "spatial query result:" << std::endl;
    for (const auto& value : result_s)
      std::cout << bg::wkt<box>(value.first) << " - " << value.second
                << std::endl;
  }

  // 検索その2：(0, 0) の点から最も近い box を5つ取得 (Nearest neighbors query)
  {
    // knn query の結果
    std::vector<value> result_n;
    // https://www.boost.org/doc/libs/1_78_0/libs/geometry/doc/html/geometry/spatial_indexes/queries.html
    rtree.query(bgi::nearest(point(0, 0), 5), std::back_inserter(result_n));

    // knn query の結果を表示
    std::cout << "knn query point:" << std::endl;
    std::cout << bg::wkt<point>(point(0, 0)) << std::endl;
    std::cout << "knn query result:" << std::endl;
    for (const auto& value : result_n)
      std::cout << bg::wkt<box>(value.first) << " - " << value.second
                << std::endl;
  }
}