#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
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

//这里我特意改成引用，是为了节省开销，实际调用的函数还是原来的复制
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*>& objects)
{
	BVHBuildNode* node = nullptr;
	switch (splitMethod)
	{
	case SplitMethod::SAH:
		node = recursiveBuildSAH(objects);
		break;
	case SplitMethod::NAIVE:
		node = recursiveBuildNaive(objects);
		break;
	default:
		break;
	}
	return node;
}

BVHBuildNode* BVHAccel::recursiveBuildNaive(std::vector<Object*>objects)
{
	BVHBuildNode* node = new BVHBuildNode();
	// Compute bounds of all primitives in BVH node
   // Bounds3 bounds;
   // for (int i = 0; i < objects.size(); ++i)
	//    bounds = Union(bounds, objects[i]->getBounds());
	if (objects.size() == 1) {
		// Create leaf _BVHBuildNode_
		node->bounds = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		return node;
	}
	else if (objects.size() == 2) {
		node->left = recursiveBuildNaive(std::vector<Object*>{objects[0]});
		node->right = recursiveBuildNaive(std::vector<Object*>{objects[1]});
		node->bounds = Union(node->left->bounds, node->right->bounds);
		return node;
	}
	else {
		Bounds3 centroidBounds;
		for (int i = 0; i < objects.size(); ++i)
			centroidBounds =
			Union(centroidBounds, objects[i]->getBounds().Centroid());
		int dim = centroidBounds.maxExtent();
		switch (dim) {
		case 0:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().x <
					f2->getBounds().Centroid().x;
			});
			break;
		case 1:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().y <
					f2->getBounds().Centroid().y;
			});
			break;
		case 2:
			std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
				return f1->getBounds().Centroid().z <
					f2->getBounds().Centroid().z;
			});
			break;
		}

		auto beginning = objects.begin();
		auto middling = objects.begin() + (objects.size() / 2);
		auto ending = objects.end();

		auto leftshapes = std::vector<Object*>(beginning, middling);
		auto rightshapes = std::vector<Object*>(middling, ending);

		assert(objects.size() == (leftshapes.size() + rightshapes.size()));

		node->left = recursiveBuildNaive(leftshapes);
		node->right = recursiveBuildNaive(rightshapes);

		node->bounds = Union(node->left->bounds, node->right->bounds);
	}

	return node;
}

BVHBuildNode* BVHAccel::recursiveBuildSAH(std::vector<Object*>objects)
{
	BVHBuildNode* node = new BVHBuildNode();
	
	if (objects.size() == 0)
	{
		std::cout << "size is zero !!!!!!!" << std::endl;
	}
	if (objects.size() == 1) {
		// Create leaf _BVHBuildNode_
		node->bounds = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		return node;
	}
	else if (objects.size() == 2) {
		node->left = recursiveBuildSAH(std::vector<Object*>{objects[0]});
		node->right = recursiveBuildSAH(std::vector<Object*>{objects[1]});
		node->bounds = Union(node->left->bounds, node->right->bounds);
		return node;
	}
	else {
		int BucketCnt = 24;
		std::vector<Vector3f> centroids;
		Vector3f centroidMin{ std::numeric_limits<float>::infinity() }, centroidMax{-std::numeric_limits<float>::infinity()};
		Bounds3 allBound;
		float allArea;
		for (int i = 0; i < objects.size(); ++i)
		{
			centroids.push_back(objects[i]->getBounds().Centroid());
			allBound = Union(allBound, centroids[i]);
			for (int dim = 0; dim < 3; ++dim)
			{
				centroidMin[dim] = fmin(centroidMin[dim], centroids[i][dim]);
				centroidMax[dim] = fmax(centroidMax[dim], centroids[i][dim]);
			}
		}
		allArea = allBound.SurfaceArea();
		std::vector<std::vector<std::vector<int>>> BucketToPrim(3, std::vector<std::vector<int>>(BucketCnt, std::vector<int>()));
		int selectDim = 0, selectBucket = 0;
		float selectCost = std::numeric_limits<float>::infinity();
		for(int dim = 0; dim < 3; ++dim)
		{
			//分配入桶
			float centroidAxisMin = centroidMin[dim], centroidAxisMax = centroidMax[dim] ;
			auto get_Bucketindex = [BucketCnt,centroidAxisMin, centroidAxisMax](float centroidAxist)
				{ return int(floor(1.0*BucketCnt*(centroidAxist - centroidAxisMin) / (centroidAxisMax - centroidAxisMin))); };
			for (int i = 0; i < objects.size(); ++i)
			{
				int t_index = std::min(get_Bucketindex(centroids[i][dim]), BucketCnt - 1);
				BucketToPrim[dim][t_index].push_back(i);
			}
			std::vector<Bounds3> leftsum, rightsum;
			std::vector<int> leftcntsum, rightcntsum;
			Bounds3 leftBound, rightBound;
			int leftCnt=0, rightCnt=0;
			//计算包围盒和数量
			for (int i = 0; i < BucketCnt-1; ++i)
			{
				for (int j = 0; j < BucketToPrim[dim][i].size(); ++j)
					leftBound = Union(leftBound, centroids[BucketToPrim[dim][i][j]]);
				leftCnt += BucketToPrim[dim][i].size();
				leftsum.push_back(leftBound);
				leftcntsum.push_back(leftCnt);

				for (int j = 0; j < BucketToPrim[dim][BucketCnt - 1 - i].size(); ++j)
					rightBound = Union(rightBound, centroids[BucketToPrim[dim][BucketCnt - 1 - i][j]]);
				rightCnt += BucketToPrim[dim][BucketCnt - 1 - i].size();
				rightsum.push_back(rightBound);
				rightcntsum.push_back(rightCnt);
			}
			//这个是SAH的计算公式
			for (int i = 0; i < BucketCnt - 1; ++i)
			{
				float t_cost = 0;
				if(leftcntsum[i]!=0)
					t_cost += (leftcntsum[i] * leftsum[i].SurfaceArea() ) ;
				if (rightcntsum[i] != 0)
					t_cost += rightcntsum[BucketCnt - 2 - i] * rightsum[BucketCnt - 2 - i].SurfaceArea();
				if (t_cost < selectCost)
				{
					selectCost = t_cost;
					selectDim = dim;
					selectBucket = i;
				}
			}

		}
		//开始迭代
		auto leftshapes = std::vector<Object*>();
		auto rightshapes = std::vector<Object*>();
		for (int i = 0; i <= selectBucket; ++i)
			for (int j : BucketToPrim[selectDim][i])
				leftshapes.push_back(objects[j]);
		for (int i = selectBucket + 1; i < BucketCnt ; ++i)
			for (int j : BucketToPrim[selectDim][i])
				rightshapes.push_back(objects[j]);
		if (leftshapes.size() == 0 || rightshapes.size() == 0)
			return  recursiveBuildNaive(objects);
		node->left = recursiveBuildSAH(leftshapes);
		node->right = recursiveBuildSAH(rightshapes);
		node->bounds = Union(node->left->bounds, node->right->bounds);
	}
	return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
	std::array<int, 3> dirIsNeg{ int(ray.direction.x > 0), int(ray.direction.y > 0), int(ray.direction.z > 0) };
	if (node==nullptr || !node->bounds.IntersectP(ray,ray.direction_inv,dirIsNeg ))
		return Intersection();
	if (node->left == nullptr && node->right == nullptr)
	{
		return node->object->getIntersection(ray);
	}
	Intersection interLeft = getIntersection(node->left, ray);
	Intersection interRight = getIntersection(node->right, ray);
//	std::cout << "BVH getinersectionLeft : " << interLeft.coords.x << "   " << interLeft.coords.y << "  " << interLeft.coords.z << std::endl;
//	std::cout << "BVH getinersectionRight : " << interRight.coords.x << "   " << interRight.coords.y << "  " << interRight.coords.z << std::endl;
	return interLeft.distance<interRight.distance?interLeft:interRight;
}