#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects)
{
    BVHBuildNode *node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    // Bounds3 bounds;
    // for (int i = 0; i < objects.size(); ++i)
    //     bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1)
    {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        std::vector<Object *> leftshapes;
        std::vector<Object *> rightshapes;
        switch (dim)
        {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().x <
                               f2->getBounds().Centroid().x; });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().y <
                               f2->getBounds().Centroid().y; });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return f1->getBounds().Centroid().z <
                               f2->getBounds().Centroid().z; });
            break;
        }
        switch (splitMethod)
        {
        case SplitMethod::NAIVE:
        {
            auto beginning = objects.begin();
            auto middling = objects.begin() + (objects.size() / 2);
            auto ending = objects.end();

            leftshapes = std::vector<Object *>(beginning, middling);
            rightshapes = std::vector<Object *>(middling, ending);

            break;
        }
        case SplitMethod::SAH:
        {
            double totalSurfaceArea = centroidBounds.SurfaceArea();
            const int bucketsSize = 10;
            std::vector<BucketUnit> Buckets;
            Buckets.resize(bucketsSize);

            std::pair<float, float> span = {objects.front()->getBounds().pMin[dim], objects.back()->getBounds().pMax[dim]};
            for (auto &obj : objects)
            {
                BucketUnit &bucket = Buckets[computeBucketIndex(obj, span, dim, bucketsSize)];
                bucket.bound = Union(bucket.bound, obj->getBounds());
                bucket.primitiveCount++;
            }
            double minCost = std::numeric_limits<float>::max();
            int index = -1;
            for (int i = 0; i < bucketsSize; ++i)
            {
                auto left = std::vector<BucketUnit>(Buckets.begin(), Buckets.begin() + i + 1);
                auto right = std::vector<BucketUnit>(Buckets.begin() + i + 1, Buckets.end());

                double SA = Object::getUnionOfObjects(left).SurfaceArea();
                double SB = Object::getUnionOfObjects(right).SurfaceArea();

                double cost = computeCost(SA, left.size(), SB, right.size(), totalSurfaceArea);
                if (cost < minCost)
                {
                    minCost = cost;
                    index = i;
                }
            }

            int offset = calculateOffset(Buckets, index);
            leftshapes = std::vector<Object *>(objects.begin(), objects.begin() + offset);
            rightshapes = std::vector<Object *>(objects.begin() + offset, objects.end());

            break;
        }
        }
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
}

Intersection BVHAccel::Intersect(const Ray &ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    Intersection inter;
    // TODO Traverse the BVH to find intersection
    if (!node)
    {
        return inter;
    }
    if (!node->bounds.IntersectP(ray, ray.direction_inv, {int(ray.direction[0] > 0), int(ray.direction[1] > 0), int(ray.direction[2] > 0)}))
    {
        return inter;
    }
    if (!node->left && !node->right)
    {
        return node->object->getIntersection(ray);
    }

    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);

    return hit1.distance < hit2.distance ? hit1 : hit2;
}

int BVHAccel::computeBucketIndex(Object *&obj, std::pair<float, float> span, int dim, const int bucketSize)
{
    float coord = (obj->getBounds().Centroid())[dim];
    if (span.first < 0)
    {
        coord += (-span.first);
    }
    else if (span.first > 0)
    {
        coord -= span.first;
    }
    float totalLength = span.second - span.first;
    float bucketLength = totalLength / bucketSize;
    return static_cast<int>(coord / bucketLength);
}

double BVHAccel::computeCost(double SA, int num_A, double SB, int num_B, double totalSurfaceArea)
{
    return (SA * num_A + SB * num_B) / totalSurfaceArea;
}

int BVHAccel::calculateOffset(std::vector<BucketUnit> &v, int index)
{
    int res = 0;
    for (int i = 0; i < index + 1; ++i)
    {
        res += v[i].primitiveCount;
    }
    return res;
}
