/**
 * cf. https://pcl.readthedocs.io/en/latest/adding_custom_ptype.html#adding-custom-ptype
 * cf. https://pcl.readthedocs.io/en/latest/kdtree_search.html#kdtree-search
 */
// テンプレート関数の読み込みのため, PCL の include の前に必要 (from PCL 1.7)
#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <random>

// アライメントのための SSE パディング
struct EIGEN_ALIGN16 MyPoint : public pcl::PointXYZ {
    float value;
    // Allocator のアライメントのため
    PCL_MAKE_ALIGNED_OPERATOR_NEW

    inline MyPoint() : pcl::PointXYZ(0, 0, 0), value(0) {}

    explicit MyPoint(pcl::PointXYZ xyz) : pcl::PointXYZ(xyz), value(0) {}
};

// テンプレート関数・クラスの実体化のために独自型を登録 (グローバルスコープで定義)
POINT_CLOUD_REGISTER_POINT_STRUCT (MyPoint,
                                   (float, x, x)(float, y, y)(float, z, z)(float, value, value))

int main() {
    // ハードウェア乱数
    std::random_device seed_gen;
    // メルセンヌ・ツイスター法による擬似乱数生成器
    std::mt19937 engine(seed_gen());
    // 一様実数分布. [0.0, 1.0)の値の範囲で等確率に実数を生成.
    std::uniform_real_distribution<> dist(0.0, 1.0);

    // 検索対象の点群
    pcl::PointCloud<MyPoint>::Ptr cloud(new pcl::PointCloud<MyPoint>);
    // unorganized (2次元整列しない) 点群設定. 点の数は 1000x1 個
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);
    // 点生成
    for (std::size_t i = 0; i < cloud->size(); ++i) {
        (*cloud)[i].x = static_cast<float>(1024.0f * dist(engine));
        (*cloud)[i].y = static_cast<float>(1024.0f * dist(engine));
        (*cloud)[i].z = static_cast<float>(1024.0f * dist(engine));
        (*cloud)[i].value = static_cast<float>(1024.0 * dist(engine));
    }

    // FLANN 実装による kd-tree
    pcl::KdTreeFLANN<MyPoint> kdtree;
    // kd-tree に点群登録
    kdtree.setInputCloud(cloud);

    // 検索の query となる点
    pcl::PointXYZ searchPoint;
    searchPoint.x = static_cast<float>(1024.0f * dist(engine));
    searchPoint.y = static_cast<float>(1024.0f * dist(engine));
    searchPoint.z = static_cast<float>(1024.0f * dist(engine));

    // K nearest neighbor search
    {
        // 検索数
        uint16_t K = 10;

        std::cout << "K nearest neighbor search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z
                  << ") with K=" << K << std::endl;

        // 検索結果のインデックスリスト
        std::vector<int> pointIdxKNNSearch(K);
        // 検索結果の距離リスト
        std::vector<float> pointKNNSquaredDistance(K);
        if (kdtree.nearestKSearch(MyPoint(searchPoint), K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0) {
            // 結果を表示
            for (const auto &index: pointIdxKNNSearch) {
                std::cout << "    (" << (*cloud)[static_cast<size_t>(index)].x
                          << ", " << (*cloud)[static_cast<size_t>(index)].y
                          << ", " << (*cloud)[static_cast<size_t>(index)].z
                          << ", " << (*cloud)[static_cast<size_t>(index)].value
                          << ") (squared distance: " << index << ")" << std::endl;
            }
        }
    }

    // Neighbors within radius search
    {
        // 検索半径
        auto radius = static_cast<float>(256.0f * dist(engine));;

        std::cout << "Neighbors within radius search at (" << searchPoint.x
                  << " " << searchPoint.y
                  << " " << searchPoint.z
                  << ") with radius=" << radius << std::endl;

        // 検索結果のインデックスリスト
        std::vector<int> pointIdxRadiusSearch;
        // 検索結果の距離リスト
        std::vector<float> pointRadiusSquaredDistance;
        if (kdtree.radiusSearch(MyPoint(searchPoint), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // 結果を表示
            for (const auto &index: pointIdxRadiusSearch) {
                std::cout << "    (" << (*cloud)[static_cast<size_t>(index)].x
                          << ", " << (*cloud)[static_cast<size_t>(index)].y
                          << ", " << (*cloud)[static_cast<size_t>(index)].z
                          << ", " << (*cloud)[static_cast<size_t>(index)].value
                          << ") (squared distance: " << index << ")" << std::endl;
            }
        }
    }

    return 0;
}