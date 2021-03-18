// glintbounce.cpp : Defines the entry point for the console application.
//
#include "localEigen/Dense"
#include "iostream"
#include "camera.h"
#include "util.h"
#include <time.h>
#include "shape.h"
#include <mitsuba/render/util.h>
#include "unsupported/Eigen/AutoDiff"
#include "intervalpath.h"
#include <queue>
#include <stack>
#include <mitsuba/render/scene.h>
#include <mitsuba/core/ray.h>

using namespace std;
using namespace Eigen;
using namespace mitsuba;

typedef Eigen::Vector3f Vec3;
typedef Eigen::Vector2i Vec2i;
typedef Eigen::Vector3i Vec3i;

MTS_NAMESPACE_BEGIN

typedef Eigen::Vector3f Vec3;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Vector2f Vec2;
typedef Eigen::Vector4f Vec4;


typedef Eigen::Matrix<float, 6, 1> Vec6;

//-----------------------------------------------------
typedef AutoDiffScalar<Eigen::Vector2f> DFloat;

typedef Eigen::Matrix<DFloat, 4, 1> DVec4;
typedef Eigen::Matrix<DFloat, 3, 1> DVec3;
typedef Eigen::Matrix<DFloat, 2, 1> DVec2;

typedef Eigen::Matrix<DFloat, 2, 1, 0, 2, 1> DMatrix2;
typedef Eigen::Matrix<float, 3, 2> Matrix3x2;
typedef Eigen::Matrix<float, 2, 3> Matrix2x3;
typedef Eigen::Matrix<float, 2, 2> Matrix2;


// we only need four derivative to track for two bounce case
//-----------------------------------------------------
typedef AutoDiffScalar<Vec4> DFloat4;
typedef Eigen::Matrix<DFloat4, 4, 1> DVec44;
typedef Eigen::Matrix<DFloat4, 6, 1> DVec46;
typedef Eigen::Matrix<DFloat4, 3, 1> DVec43;
typedef Eigen::Matrix<DFloat4, 2, 1> DVec42;
typedef Eigen::Matrix<float, 6, 4> Matrix6x4;
typedef Eigen::Matrix<float, 4, 6> Matrix4x6;


//for three bounce
typedef Eigen::Matrix<float, 6, 1> Vec6;
typedef AutoDiffScalar<Vec6> DFloat6;
typedef Eigen::Matrix<DFloat6, 6, 1> DVec66;
typedef Eigen::Matrix<DFloat6, 9, 1> DVec69;
typedef Eigen::Matrix<DFloat6, 3, 1> DVec63;
typedef Eigen::Matrix<DFloat6, 2, 1> DVec62;

typedef Eigen::Matrix<float, 6, 9> Matrix6x9;
typedef Eigen::Matrix<float, 9, 6> Matrix9x6;

//for four bounce
typedef Eigen::Matrix<float, 8, 1> Vec8;
typedef AutoDiffScalar<Vec8> DFloat8;
typedef Eigen::Matrix<DFloat8, 8, 1> DVec88;
typedef Eigen::Matrix<DFloat8, 12, 1> DVec8_12;
typedef Eigen::Matrix<DFloat8, 3, 1> DVec83;

typedef Eigen::Matrix<float, 8, 12> Matrix8x12;
typedef Eigen::Matrix<float, 12, 8> Matrix12x8;

//for five bounce
typedef Eigen::Matrix<float, 10, 1> Vec10;
typedef AutoDiffScalar<Vec10> DFloat10;
typedef Eigen::Matrix<DFloat10, 10, 1> DVec10_10;
typedef Eigen::Matrix<DFloat10, 15, 1> DVec10_15;
typedef Eigen::Matrix<DFloat10, 3, 1> DVec10_3;

typedef Eigen::Matrix<float, 10, 15> Matrix10x15;
typedef Eigen::Matrix<float, 15, 10> Matrix15x10;

typedef Eigen::Matrix<float, 2, 2> Matrix2;
#define M_PI  3.14159265358979323846f

enum ELightTransportMode
{
	ER = 0,
	ET,
	ERR,
	ETT,
	ETRT
}; 


class BounceGlintRenderer{
public:
	//for shapes with height field
	BounceGlintRenderer(const Scene *scene, string hfPath, float scale)
	{
		m_errorThresh = 0.001;
		HFMesh = true;

		m_scene = scene;
		//set emitter
		setEmitter(scene);
		//for camera
		setCamera(scene);

		ref_vector<Shape> shapes = scene->getShapes();
		for (int i = 0; i < shapes.size(); i++)
		{
			const Shape *shape = shapes[i];
			if (shape->getEmitter() != NULL)
				continue;
			//make sure the shape is a quad.
			Point center = shape->getAABB().getCenter();
			const TriMesh *mesh = static_cast<const TriMesh *>(shape);
			const Point *positions = mesh->getVertexPositions();
			//we only have to check on triangle for the normal
			Point p1 = positions[0];
			Point p2 = positions[1];
			Point p3 = positions[2];
			Vec3 faceNormal = getNormal(Vec3(p1[0], p1[1], p1[2]), Vec3(p3[0], p3[1], p3[2]), Vec3(p2[0], p2[1], p2[2]));
			cout << "shapeNor:" << faceNormal[0] << "," << faceNormal[1] << "," << faceNormal[2] << endl;
			cout << "shapePos:" << center[0] << "," << center[1] << "," << center[2] << endl;
			float length = sqrt(shape->getSurfaceArea());
			cout << "Length:" << length << endl;
			float maxHeight = 0.015 * scale;// 0.015;// 0.03;
			cout << "MaxHeight:" << maxHeight << endl;
			const BSDF *bsdf = shape->getBSDF();
			
			string hfName = hfPath;// bsdf->getProperties().getString("inputHF");
			MyBSDF *mybsdf1;
			MyBSDF *mybsdf2;
			if (bsdf->hasComponent(BSDF::EDeltaTransmission))
			{
				float eta = bsdf->getEta();				
				Spectrum transmittanceColor = bsdf->getProperties().getSpectrum("specularTransmittance", Spectrum(1.0));
				mybsdf1 = new DilectricRefraction(eta, Vec3(transmittanceColor[0], transmittanceColor[1], transmittanceColor[2]));
				mybsdf2 = new DilectricReflection(eta); //eta is used for F computation
				cout << "Trnsmittance Eta:" << eta << endl;
			}
			else if (bsdf->hasComponent(BSDF::EDeltaReflection))
			{
				mybsdf1 = new Reflection(bsdf->getProperties().getString("material"));
				mybsdf2 = NULL;
				cout << "Use Reflectance" << endl;
			}
			else
			{
				cout << "The BSDF type of the shape is invalid for glint." << endl;
				continue;
			}

			MyShape *myshape = new MyShape(Vec3(center.x, center.y, center.z), hfName, length, faceNormal,
				shape, maxHeight, mybsdf1, mybsdf2, m_camera->getPos(), m_scene);// true, m_optimizor); //
			m_shapes.push_back(myshape);
			cout << "Add one shape..." << endl;
		}
		outputSomeInform();

	};


	//the input is a mesh from XML
	BounceGlintRenderer(const Scene *scene)
	{
		m_errorThresh = 0.00001;

		HFMesh = false;
		//assume there is only one emitter
		m_scene = scene;

		//set emitter
		setEmitter(scene);
		//for camera
		setCamera(scene);

		ref_vector<Shape> shapes = scene->getShapes();
		for (int i = 0; i < shapes.size(); i++)
		{
			const Shape *shape = shapes[i];
			if (shape->getEmitter() != NULL)
				continue;

			const BSDF *bsdf = shape->getBSDF();
			MyBSDF *mybsdf1;
			MyBSDF *mybsdf2;
			if (bsdf->hasComponent(BSDF::EDeltaTransmission))
			{
				float eta = bsdf->getEta();
				Spectrum transmittanceColor = bsdf->getProperties().getSpectrum("specularTransmittance", Spectrum(1.0));
				mybsdf1 = new DilectricRefraction(eta, Vec3(transmittanceColor[0], transmittanceColor[1], transmittanceColor[2]));
				mybsdf2 = new DilectricReflection(eta);
				cout << "Trnsmittance Eta:" << eta << endl;

			}
			else if (bsdf->hasComponent(BSDF::EDeltaReflection))
			{
				mybsdf1 = new Reflection(bsdf->getProperties().getString("material"));
				mybsdf2 = NULL;
				cout << "Use Reflectance" << endl;
			}
			else
			{
				cout << "The BSDF type of the shape is invalid for glint." << endl;
				continue;
			}

			MyShape *myshape = new MyShape(shape, mybsdf1, mybsdf2, m_camera->getPos(), m_scene);
			m_shapes.push_back(myshape);
			cout << "Add one shape..." << endl;
		}
		outputSomeInform();
	};


	~BounceGlintRenderer()
	{
		delete m_camera;
		for (int i = 0; i < m_shapes.size(); i++)
		{
			delete m_shapes[i];
		}
		std::vector<MyShape*>().swap(m_shapes);
	};

	void outputSomeInform()
	{
		{
			Vec3 camPos = m_camera->getPos();
			Vec3 camDir = m_camera->getDir();
			cout << "CamPos:" << camPos[0] << "," << camPos[1] << "," << camPos[2] << endl;
			cout << "LightPos:" << m_lightPos[0] << "," << m_lightPos[1] << "," << m_lightPos[2] << endl;
			cout << "CamDir:" << camDir[0] << "," << camDir[1] << "," << camDir[2] << endl;
		}

		int totalTriangle = 0;
		for (int k = 0; k < m_shapes.size(); k++)
		{
			const int triaCount1 = m_shapes[k]->m_triangles.size();
			totalTriangle += triaCount1;
			cout << "Shape " << k << ":  " << triaCount1 << endl;
		}
		cout << "Total Triangle Count:  " << totalTriangle << endl;
		{
			long long memoryCost = 0;
			for (int k = 0; k < m_shapes.size(); k++)
			{
				memoryCost += m_shapes[k]->getHiearchySize();
			}
			float memCost = memoryCost / (1024.0f * 1024.0f);
			cout << "The memory cost is: " << memCost << "MB" << endl;
		}
	}

	void setEmitter(const Scene *scene)
	{
		//assume there is only one emitter
		const Emitter * emitter = NULL;
		m_lightCount = 0;
		for (int i = 0; i < scene->getEmitters().size(); i++)
		{
			const Emitter *e = scene->getEmitters()[i].get();
			if (e->getType() == Emitter::EDeltaPosition)
			{
				emitter = e;
				const Transform &trafo = e->getWorldTransform()->eval(0);
				Point pos = trafo(Point(0.0f));
				m_lightPos = Vec3(pos.x, pos.y, pos.z);
				//for light intensity
				Spectrum intensity = e->getProperties().getSpectrum("intensity");
				m_lightIntensity = Vec3(intensity[0], intensity[1], intensity[2]);

				m_lightCount++;
				m_lightPosList.push_back(m_lightPos);
				m_lightIntensList.push_back(m_lightIntensity);
			}
			else if (e->getType() == Emitter::EOnSurface)
			{
				emitter = e;
				Point pos = e->getShape()->getAABB().getCenter();
				m_lightPos = Vec3(pos.x, pos.y, pos.z);
				//for light intensity
				float surArea = e->getShape()->getSurfaceArea();
				//we also need to consider the area,
				//from area intensity to point light intensity
				Spectrum intensity = surArea * e->getProperties().getSpectrum("radiance");
				m_lightIntensity = Vec3(intensity[0], intensity[1], intensity[2]);
			}
			else if (e->isEnvironmentEmitter())
			{
				m_env = e;
			}
		}
		if (emitter == NULL)
		{
			cout << "No valid emitter is specified!" << endl;
			m_lightPos = Vec3(-20.0f, 0.0f, 20.0f);
		}

	}

	void setOnePointEmitter( int i)
	{
		//assume there is only one emitter
		m_lightPos = m_lightPosList[i];
		m_lightIntensity = m_lightIntensList[i];
	}

	void setCamera(const Scene *scene)
	{
		const Sensor *cam = scene->getSensor();
		const Transform &transf = cam->getWorldTransform()->eval(0);
		Point p = transf(Point(0, 0, 0));
		Vector d = transf(Vector(0, 0, 1));

		Vec3 camPos(p.x, p.y, p.z);
		Vec3 camDir(d.x, d.y, d.z);

		const Matrix4x4 &m = transf.getMatrix();
		Vec3 up = Vec3(m(0, 1), m(1, 1), m(2, 1));
		Vec3 left = Vec3(m(0, 0), m(1, 0), m(2, 0));

		float fov = cam->getProperties().getFloat("fov", 10.0);
		Vector2i res = cam->getFilm()->getSize();

		const Vector2i &filmSize = cam->getFilm()->getSize();
		const Vector2i &cropSize = cam->getFilm()->getCropSize();
		const Point2i  &cropOffset = cam->getFilm()->getCropOffset();

		Vector2 relSize((Float)cropSize.x / (Float)filmSize.x,
			(Float)cropSize.y / (Float)filmSize.y);
		Point2 relOffset((Float)cropOffset.x / (Float)filmSize.x,
			(Float)cropOffset.y / (Float)filmSize.y);

		float aspect = cam->getAspect();
		Transform m_cameraToSample =
			Transform::scale(Vector(1.0f / relSize.x, 1.0f / relSize.y, 1.0f))
			* Transform::translate(Vector(-relOffset.x, -relOffset.y, 0.0f))
			* Transform::scale(Vector(-0.5f, -0.5f*aspect, 1.0f))
			* Transform::translate(Vector(-1.0f, -1.0f / aspect, 0.0f))
			* Transform::perspective(((PerspectiveCamera*)cam)->getXFov(), ((ProjectiveCamera*)cam)->getNearClip(), ((ProjectiveCamera*)cam)->getFarClip());

		m_camera = new Camera(camPos, camDir, up, left, Vec2i(res.x, res.y), fov, m_cameraToSample);

	}


	float evalError(const DVec46 &fx)
	{
		float error = 0.0f;
		for (int i = 0; i < 6; i++)
		{
			error += abs(fx[i].value());

		}
		return error;
	}
	float evalError(const Vec6 &fx)
	{
		float error = 0.0f;
		for (int i = 0; i < 6; i++)
		{
			error += abs(fx[i]);

		}
		return error;
	}
	float evalError(const Vec3 &fx)
	{
		float error = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			error += abs(fx[i]);

		}
		return error;
	}
	float evalError(const DVec69 &fx)
	{
		float error = 0.0f;
		for (int i = 0; i < 9; i++)
		{
			error += abs(fx[i].value());

		}
		return error;
	}
	float evalError(const DVec3 &fx)
	{
		float error = 0.0f;
		for (int i = 0; i < 3; i++)
		{
			error += abs(fx[i].value());

		}
		return error;
	}
	float evalError(const DVec8_12 &fx)
	{
		float error = 0.0f;
		for (int i = 0; i < 12; i++)
		{
			error += abs(fx[i].value());

		}
		return error;
	}
	float evalError(const DVec10_15 &fx)
	{
		float error = 0.0f;
		for (int i = 0; i < 15; i++)
		{
			error += abs(fx[i].value());

		}
		return error;
	}

	//the functions fro Newton solver
	//<DFloat6, DVec69, Vec6, Matrix9x6, Matrix6x9,DVec66>
	template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
	T2 f_Bounce(T1 alpha[], T1 beta[],
		Vec3 v[][3], Vec3 vn[][3],
		int shape[], int bounce, const Vec3 &viewPos, float eta[])
	{
		T1 gamma[MAX_Bounce];

		for (int i = 0; i < bounce; i++)
		{
			gamma[i] = 1 - alpha[i] - beta[i];

		}

		T7 p[MAX_Bounce], n[MAX_Bounce];
		//this should bounce + 2, the first for camera, and the last for light
		for (int k = 0; k < bounce; k++)
		{
			p[k + 1] = alpha[k] * v[k][0] + beta[k] * v[k][1] + gamma[k] * v[k][2];
			n[k + 1] = (alpha[k] * vn[k][0] + beta[k] * vn[k][1] + gamma[k] * vn[k][2]).normalized();

		}
		p[0] = viewPos;
		p[bounce + 1] = m_lightPos;
		T2 result; //DVec69
		for (int k = 1; k < bounce + 1; k++)
		{
			T7 in = (p[k - 1] - p[k]).normalized();
			T7 out = (p[k + 1] - p[k]).normalized();

			//eta is on the inside direction
			float realEta = eta[k];
			if (in.dot(n[k]) < 0)
				realEta = 1.0 / realEta;

			T7 h = (in + realEta*out).normalized();
			T7 d = n[k] + m_shapes[shape[k - 1]]->m_bsdf1->getSign() * h;
			result[3 * (k - 1)] = d[0];
			result[3 * (k - 1) + 1] = d[1];
			result[3 * (k - 1) + 2] = d[2];
		}

		return result;
	}

	
	//<DFloat6, DVec69, Vec6, Matrix9x6, Matrix6x9,DVec66,DVec63>
	template<typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7>
	int NewtonSolver(int iShape[], int iTriangle[],
		std::vector<Path *> &path, int &pathcutAfterNewton, const int bounce, int mode[], float eta[])
	{
		T1 alpha1[MAX_Bounce];  //DFloat6
		T1 beta1[MAX_Bounce];
		Vec3i idx[MAX_Bounce];
		Vec3 v[MAX_Bounce][3];
		Vec3 vn[MAX_Bounce][3];
		//	Vec3 vc[MAX_Bounce];
		T1 alpha[MAX_Bounce], beta[MAX_Bounce];

		const float inv_three = 1.0f / 3.0f;
		const int dCount = 2 * bounce;
		for (int i = 0; i < bounce; i++)
		{
			MyShape *shape = m_shapes[iShape[i]];
			alpha1[i] = T1(inv_three, dCount, 2 * i);
			beta1[i] = T1(inv_three, dCount, 2 * i + 1);
			alpha[i] = T1(inv_three, dCount, 2 * i);
			beta[i] = T1(inv_three, dCount, 2 * i + 1);
			Vec3i iIndex = shape->m_triangles[iTriangle[i]].index;

			const vector<Vec3> &verts = shape->m_verts;
			const vector<Vec3> &vns = shape->m_vertexNormal;
			for (int k = 0; k < 3; k++)
			{
				v[i][k] = verts[iIndex[k]];
				vn[i][k] = vns[iIndex[k]];

			}
		}

		T2 fx; //DVec69
		const int itrCount = 5;
		bool found = false;
		const int largeD = 3 * bounce;
		const int shortD = 2 * bounce;


		for (int i = 0; i < itrCount; i++)
		{
			bool valid = true;
			for (int k = 0; k < bounce; k++)
			{
				alpha[k] = T1(alpha1[k].value(), dCount, (2 * k));
				beta[k] = T1(beta1[k].value(), dCount, (2 * k + 1));

				if (alpha[k].value() > 10 || alpha[k].value() < -10 || beta[k].value() > 10 || beta[k].value() < -10)
				{
					valid = false;
					break;
				}
			}
			if (!valid)
			{
				found = false;
				break;
			}
			fx = f_Bounce<T1, T2, T3, T4, T5, T6, T7>(alpha, beta, v, vn, iShape, bounce, m_camera->getPos(), eta);
			//fx = f_Bounce_constH<T1, T2, T3, T4, T5, T6, T7>(alpha, beta, vn, vc, shape, bounce, m_camera->getPos(), eta);

			float error = evalError(fx);
			if (error < m_errorThresh)
			{
				found = true;
				break;
			}

			//9:3*bounce
			T3 jacobian[3 * MAX_Bounce]; //Vec6
			for (int k = 0; k < largeD; k++)
				jacobian[k] = fx[k].derivatives();

			T4 J; //Matrix9x6
			for (int j = 0; j < shortD; j++)
			{
				for (int k = 0; k < largeD; k++)
				{
					J(k, j) = jacobian[k][j];
				}

			}

			T5 Jt = J.transpose();  ////Matrix6x9
			T5 invJ = (Jt * J).inverse()*Jt;

			T6 alpha_beta = invJ *fx; //DVec66

			for (int k = 0; k < bounce; k++)
			{
				alpha1[k] = alpha[k] - alpha_beta[2 * k];
				beta1[k] = beta[k] - alpha_beta[2 * k + 1];
			}
		}

		if (!found)
			return 0;

		float a[MAX_Bounce], b[MAX_Bounce], c[MAX_Bounce];
		Vec3 p[MAX_Bounce], n[MAX_Bounce];
		for (int k = 0; k < bounce; k++)
		{
			a[k] = alpha[k].value();
			b[k] = beta[k].value();
			if (a[k] < 0 || b[k] <0 || a[k] + b[k] > 1)
				return 0;
			c[k] = 1.0f - a[k] - b[k];
			p[k] = v[k][0] * a[k] + v[k][1] * b[k] + v[k][2] * c[k];
			n[k] = vn[k][0] * a[k] + vn[k][1] * b[k] + vn[k][2] * c[k];
		}
		vector<PathVert> vertList(bounce + 2);

		vertList[0] = (PathVert(m_lightPos, (p[bounce-1] - m_lightPos).normalized(), m_lightIntensity)); //the light		
		vertList[bounce + 1] = PathVert(m_camera->getPos(), Vec3(0, 0, 0), Vec3(1, 1, 1)); //the camera

		for (int k = 1; k < bounce + 1; k++)
		{
			int i = bounce - k;
			MyShape *shape = m_shapes[iShape[i]];
			MyBSDF *bsdf = mode[i + 1] == -2 ? shape->m_bsdf2 : shape->m_bsdf1;

			vertList[k] = PathVert(p[i], n[i], v[i][0], v[i][1], v[i][2], vn[i][0], vn[i][1], vn[i][2],
				iShape[i], iTriangle[i], shape->m_triangles[iTriangle[i]].normal, bsdf);
		}

		path.push_back(new Path(vertList));
		pathcutAfterNewton++;

		//occlusion test
		if (!HFMesh)
		{
			bool block = m_camera->pathOcclusion(path[path.size() - 1], m_scene);
			if (block)
			{
				path.pop_back();
				return 0;
			}
		}

		return 1;
	}


	void processPathCut(const IntervalPath &current, int *iShape, int *sign, float *eta,
		std::vector<Path *> &pathListThread,
		int &pathcutVisitedN, int &pathcutVisitedLeafN, int &pathcutAfterNewton, int &connectionCout)
	{
		pathcutVisitedN++;

		int iPatch = current.findThickestArea();
		if (iPatch == -1)
		{

			int bounce = current.getLength() - 2;

			int triangleIndex[MAX_Bounce];
			for (int k = 0; k < bounce; k++)
			{
				triangleIndex[k] = current.getPatch(bounce - k)->triangleIndex;
			}

			pathcutVisitedLeafN++;

			for (int k = 0; k < bounce - 1; k++)
			{
				if (iShape[k] == iShape[k + 1] && triangleIndex[k] == triangleIndex[k + 1])
					return;
			}

			switch (bounce) {
			case 1:
				connectionCout += NewtonSolver<DFloat, DVec3, Vec2, Matrix3x2, Matrix2x3, DVec2, DVec3>(
					iShape, triangleIndex,	pathListThread, pathcutAfterNewton,
					bounce, sign, eta);

				break;
			case 2:
				connectionCout += NewtonSolver<DFloat4, DVec46, Vec4, Matrix6x4, Matrix4x6, DVec44, DVec43>(
					iShape, triangleIndex, pathListThread, pathcutAfterNewton,
					bounce, sign, eta);

				break;
			case 3:
				connectionCout += NewtonSolver<DFloat6, DVec69, Vec6, Matrix9x6, Matrix6x9, DVec66, DVec63>(
					iShape, triangleIndex, pathListThread, pathcutAfterNewton,
					bounce, sign, eta);//TRT
				break;
			case 4:
				connectionCout += NewtonSolver<DFloat8, DVec8_12, Vec8, Matrix12x8, Matrix8x12, DVec88, DVec83>(
					iShape, triangleIndex, pathListThread, pathcutAfterNewton,
					bounce, sign, eta);//TTTT
				break;
			case 5:
				connectionCout += NewtonSolver<DFloat10, DVec10_15, Vec10, Matrix15x10, Matrix10x15, DVec10_10, DVec10_3>(
					iShape, triangleIndex, pathListThread, pathcutAfterNewton,
					bounce, sign, eta);
				break;
			}

		}
		else{

			IntervalPath subPaths[4];
			current.subdivide(iPatch, subPaths);

			for (int i = 0; i < 4; i++)
			{
				const IntervalPath &subpath = subPaths[i];
				//check can we discard this path?
				bool isValid = subpath.isValid(sign, eta);

				if (isValid)
					processPathCut(subpath, iShape, sign, eta, pathListThread,
					pathcutVisitedN, pathcutVisitedLeafN, pathcutAfterNewton, connectionCout);

			}
		}
	}

	void findGlintsPathCut(std::vector<Path*> &pathList,int bounce)
	{
		std::vector<std::vector<Vec3>> path;
		std::vector<std::vector<Path*>> pathListThread;

		const int threadCount = omp_get_max_threads();
		path.resize(threadCount);
		pathListThread.resize(threadCount);

		std::vector<int> connectionCount;
		std::vector<int> pathcutVisitedN;
		std::vector<int> pathcutVisitedLeafN;
		std::vector<int> pathcutAfterNewton;
		connectionCount.resize(threadCount);
		pathcutVisitedN.resize(threadCount);
		pathcutVisitedLeafN.resize(threadCount);
		pathcutAfterNewton.resize(threadCount);
		std::vector<int> countTriangles(threadCount);
		for (int i = 0; i < threadCount; i++)
		{
			connectionCount[i] = 0;
			pathcutVisitedN[i] = 0;
			pathcutAfterNewton[i] = 0;
			pathcutVisitedLeafN[i] = 0;
			countTriangles[i] = 0;
		}

		TreeNode *lightNode = new TreeNode;
		TreeNode *camNode = new TreeNode;
		lightNode->posBox = Interval3D(m_lightPos);
		lightNode->center = m_lightPos;

		camNode->posBox = Interval3D(m_camera->getPos());
		camNode->center = m_camera->getPos();
		for (int k = 0; k < 4; k++)
		{
			camNode->child[k] = NULL;
			lightNode->child[k] = NULL;
		}

		long long bruteForceVisitedN = 0;
		//should start from light side, and include both light and camera
		int *sign = new int[bounce + 2];
		float *eta = new float[bounce + 2];
		int *iShape = new int[bounce];
		sign[0] = sign[bounce + 1] = 0;
		eta[0] = eta[bounce + 1] = 1.0;

		if (bounce == 1)
		{
			for (int is = 0; is < m_shapes.size(); is++)
			{
				bruteForceVisitedN += m_shapes[is]->m_triangles.size();

				std::vector<TreeNode*> camPatch;
				m_shapes[is]->m_root->getNodeLevel(0, 8, camPatch);
				int size = camPatch.size();
				iShape[0] = is;
				//For Beibei: can not simulate reflection for dielectric surface for now.
				sign[1] = m_shapes[is]->m_bsdf1->getSign();
				eta[1] = m_shapes[is]->m_bsdf1->getEta();


#pragma omp parallel for schedule(dynamic)
				for (int i = 0; i < size; i++)
				{
					int tid = omp_get_thread_num();
					TreeNode* patches[3];
					patches[0] = lightNode;
					patches[1] = camPatch[i];
					patches[2] = camNode;
					IntervalPath root(3,patches);
					
					processPathCut(root, iShape, sign, eta, pathListThread[tid],
						pathcutVisitedN[tid], pathcutVisitedLeafN[tid], pathcutAfterNewton[tid], connectionCount[tid]);
				}
			}
		}
		else if (bounce == 2)
		{
			ELightTransportMode mode = ERR;

			if (mode == ETT)
				cout << "Simulate light transport TT. \n";
			else
				cout << "Simulate light transport RR. \n";
			
			for (int is = 0; is < m_shapes.size(); is++)
			{
				for (int js = 0; js < m_shapes.size(); js++)
				{
					//if (is != js)
					//	continue;

					if (m_shapes[is]->m_bsdf1->getType() == MyBSDF::ERefelction
						&& m_shapes[js]->m_bsdf1->getType() == MyBSDF::ERefelction)
					{
						cout << "Simulate light transport RR." << is << "," << js <<" \n";

					}

					bruteForceVisitedN += m_shapes[js]->m_triangles.size() * m_shapes[is]->m_triangles.size();
					//start from the ligth side
					std::vector<TreeNode*> lightPatch;
					m_shapes[js]->m_root->getNodeLevel(0, 4, lightPatch);
					std::vector<TreeNode*> camPatch;
					m_shapes[is]->m_root->getNodeLevel(0, 4, camPatch);
					int size = lightPatch.size() * camPatch.size();
					
					iShape[0] = is;
					iShape[1] = js;
					//For Beibei: can not simulate reflection for dielectric surface for now.
					sign[1] = m_shapes[is]->m_bsdf1->getSign();
					sign[2] = m_shapes[js]->m_bsdf1->getSign();
					eta[1] = m_shapes[is]->m_bsdf1->getEta();
					eta[2] = m_shapes[js]->m_bsdf1->getEta();


#pragma omp parallel for schedule(dynamic)
					for (int i = 0; i < size; i++)
					{
						int n1 = i / lightPatch.size();
						int n2 = i % lightPatch.size();
						int tid = omp_get_thread_num();
						TreeNode* patches[4];
						patches[0] = lightNode;
						patches[1] = lightPatch[n2];
						patches[2] = camPatch[n1];
						patches[3] = camNode;
						IntervalPath root(4, patches);
						processPathCut(root, iShape, sign, eta, pathListThread[tid],
							pathcutVisitedN[tid], pathcutVisitedLeafN[tid], pathcutAfterNewton[tid], connectionCount[tid]);

					}

				}
			}

		}
		else if (bounce == 3)
		{
			
			cout << "Simulate light transport TRT or RRR. \n";
			//int is = 0; int js = 1; int ks = 0;
			for (int is = 0; is < m_shapes.size(); is++)
			{
				for (int js = 0; js < m_shapes.size(); js++)
				{
					for (int ks = 0; ks < m_shapes.size(); ks++)
					{

						if (m_shapes[is]->m_bsdf1->getType() == MyBSDF::ERefelction
							&& m_shapes[ks]->m_bsdf1->getType() == MyBSDF::ERefelction
							&& m_shapes[js]->m_bsdf1->getType() == MyBSDF::ERefelction)
						{
							cout << "Simulate light transport RRR." << is << "," << js << "," << ks << " \n";
							if (is == js || js == ks)
								continue;
						}
						//TRT
						else if (m_shapes[is]->m_bsdf1->getType() != MyBSDF::EDielectricRefracion
							|| m_shapes[ks]->m_bsdf1->getType() != MyBSDF::EDielectricRefracion
							|| (m_shapes[js]->m_bsdf1->getType() != MyBSDF::ERefelction && m_shapes[js]->m_bsdf2->getType() != MyBSDF::EDielectricReflection))
							return;//continue;
#if  0
						//
						else{

							if (is != js || js != ks)//for same object internal TRT, stained glass
								continue;
						}
#endif

						iShape[0] = is;
						iShape[1] = js;
						iShape[2] = ks;

						//the middle one is reflection
						sign[1] = m_shapes[is]->m_bsdf1->getSign();
						if (m_shapes[js]->m_bsdf1->getType() == MyBSDF::ERefelction)
							sign[2] = -1;
						else if (m_shapes[js]->m_bsdf2->getType() == MyBSDF::EDielectricReflection)
						{
							sign[2] = -2;
						}
						else{
							cout << "The BSDF type is not correct for the R in TRR. \n";
						}

						sign[3] = m_shapes[ks]->m_bsdf1->getSign();

						eta[1] = m_shapes[is]->m_bsdf1->getEta();
						eta[2] = 1.0;
						eta[3] = m_shapes[ks]->m_bsdf1->getEta();

						const int level[] = { 2, 2, 6 };
						std::vector<TreeNode*> patchList[MAX_Bounce];
						long long size = 1;
						int length[MAX_Bounce];

						for (int i = 0; i < bounce; i++)
						{
							int iS = iShape[bounce - i - 1];
							m_shapes[iS]->m_root->getNodeLevel(0, level[i], patchList[i]);
							size *= patchList[i].size();
							length[i] = patchList[i].size();
						}
						bruteForceVisitedN += size;

#pragma omp parallel for schedule(dynamic)
						for (int i = 0; i < size; i++)
						{
							int n[MAX_Bounce];
							n[0] = i / (length[1] * length[2]); //for light
							int n2n3 = i % (length[1] * length[2]);
							n[1] = n2n3 / (length[2]);
							n[2] = n2n3 % length[2];
							int tid = omp_get_thread_num();

							TreeNode* patches[5];
							patches[0] = lightNode;
							patches[bounce + 1] = camNode;

							for (int k = 0; k < bounce; k++)
							{
								patches[k + 1] = patchList[k][n[k]];
							}
							patches[0] = lightNode;

							IntervalPath root(5, patches);

							processPathCut(root, iShape, sign, eta, pathListThread[tid],
								pathcutVisitedN[tid], pathcutVisitedLeafN[tid], pathcutAfterNewton[tid], connectionCount[tid]);

						}
					}

				}
			}
		}
		else if (bounce == 4)
		{

			cout << "Simulate light transport TTTT or RRRR. \n";
			//int is = 0; int js = 1; int ks = 2; int fs = 3;
			int shapeIndex[] = { 0, 1, 2, 3 };
			bool RRRR = true;
			for (int i = 0; i < bounce; i++)
			{
				iShape[i] = shapeIndex[i];
				if (m_shapes[shapeIndex[i]]->m_bsdf1->getType() != MyBSDF::ERefelction)
					RRRR = false;
			}
			if (RRRR)
			{
				cout << "Simulate light transport RRRR." << shapeIndex[0] << "," << shapeIndex[1]
					<< "," << shapeIndex[2] << "," << shapeIndex[3] << " \n";
			}

			for (int i = 0; i < bounce; i++)
			{
				sign[i+1] = m_shapes[iShape[i]]->m_bsdf1->getSign();
				eta[i+1] = m_shapes[iShape[i]]->m_bsdf1->getEta();
			}

			const int level[] = { 2, 2, 2, 6 };
			std::vector<TreeNode*> patchList[MAX_Bounce];
			long long size = 1;
			int length[MAX_Bounce];

			for (int i = 0; i < bounce; i++)
			{
				int iS = shapeIndex[bounce - i - 1];
				m_shapes[iS]->m_root->getNodeLevel(0, level[i], patchList[i]);
				size *= patchList[i].size();
				length[i] = patchList[i].size();
			}
			cout << "We have " << size << " intial path cut.\n";
			//int length = patchList[0].size();
			bruteForceVisitedN += size;


#pragma omp parallel for schedule(dynamic)
			for (long long i = 0; i < size; i++)
			{
				int n[MAX_Bounce];
				n[0] = i / (length[1] * length[2] * length[3]); //for light
				int n2n3n4 = i % (length[1] * length[2] * length[3]);
				n[1] = n2n3n4 / (length[2] * length[3]); // mp2
				int n3n4 = n2n3n4 % (length[2] * length[3]);// 
				n[2] = n3n4 / length[3];// mp1
				n[3] = n3n4 % length[3];// camera

				int tid = omp_get_thread_num();

				TreeNode* patches[6];
				patches[0] = lightNode;
				patches[bounce + 1] = camNode;

				for (int k = 0; k < bounce; k++)
				{
					patches[k + 1] = patchList[k][n[k]];
				}
				IntervalPath root(6, patches);

				processPathCut(root, iShape, sign, eta, pathListThread[tid],
					pathcutVisitedN[tid], pathcutVisitedLeafN[tid], pathcutAfterNewton[tid], connectionCount[tid]);
			}
		}

		else if (bounce == 5)
		{

			cout << "Simulate light transport TTRTT. \n";
			int shapeIndex[] = { 0, 1, 2, 1, 0 };

			for (int i = 0; i < bounce; i++)
			{
				iShape[i] = shapeIndex[i];
				sign[i + 1] = m_shapes[iShape[i]]->m_bsdf1->getSign();
				eta[i + 1] = m_shapes[iShape[i]]->m_bsdf1->getEta();
			}
			const int level = 3;
			std::vector<TreeNode*> patchList[MAX_Bounce];
			int size = 1;
			int length[MAX_Bounce];
			for (int i = 0; i < bounce; i++)
			{
				int is = shapeIndex[bounce - i - 1];
				m_shapes[is]->m_root->getNodeLevel(0, level, patchList[i]);
				size *= patchList[i].size();
				length[i] = patchList[i].size();;
			}

			bruteForceVisitedN += size;

#pragma omp parallel for schedule(dynamic)
			for (int i = 0; i < size; i++)
			{
				int n[5];
				n[0] = i / (length[1] * length[2] * length[3] * length[4]); //for light
				int n2n3n4n5 = i % (length[1] * length[2] * length[3] * length[4]);
				n[1] = n2n3n4n5 / (length[2] * length[3] * length[4]); // mp3
				int n3n4n5 = n2n3n4n5 % (length[2] * length[3] * length[4]);// 
				n[2] = n3n4n5 / (length[3] * length[4]);// mp2
				int n4n5 = n3n4n5 % (length[3] * length[4]);
				n[3] = n4n5 / (length[4]);// mp1
				n[4] = n4n5 % (length[4]);// camera

				int tid = omp_get_thread_num();

				TreeNode* patches[7];
				patches[0] = lightNode;
				patches[bounce + 1] = camNode;

				for (int k = 0; k < bounce; k++)
				{
					patches[k + 1] = patchList[k][n[k]];
				}
				IntervalPath root(7, patches);
				processPathCut(root, iShape, sign, eta, pathListThread[tid],
					pathcutVisitedN[tid], pathcutVisitedLeafN[tid], pathcutAfterNewton[tid], connectionCount[tid]);

			}
		}

		clock_t t;
		t = clock();	

		for (int j = 0; j < pathListThread[0].size(); j++)
			pathList.push_back(pathListThread[0][j]);

		for (int i = 1; i < threadCount; i++)
		{
			
			connectionCount[0] += connectionCount[i];
			pathcutAfterNewton[0] += pathcutAfterNewton[i];
			pathcutVisitedN[0] += pathcutVisitedN[i];
			pathcutVisitedLeafN[0] += pathcutVisitedLeafN[i];
			countTriangles[0] += countTriangles[i];
			for (int j = 0; j < pathListThread[i].size(); j++)
			{
				pathList.push_back(pathListThread[i][j]);
			}

		}
		t = clock() - t;
		
		cout << "Brute Force Visited N: " << bruteForceVisitedN << endl;
		cout << "Visited Path cut N: " << pathcutVisitedN[0] << endl;
		cout << "Visited Leaf Path cut N: " << pathcutVisitedLeafN[0] << endl;
		cout << "After Newton solving path cut N: " << pathcutAfterNewton[0] << endl;
		cout << "Found connections: " << connectionCount[0] << endl;
		cout << "Visible Triangle count: " << countTriangles[0] << endl;

		delete eta;
		delete sign;
		delete camNode;
		delete lightNode;
	}


	void renderOneImage(string outImageName, int bounce, float errorThresh)
	{
		m_errorThresh = errorThresh;
		cout << "Start rendering..." << endl;
		cout << "Have " << abs(bounce) << " bounce." << endl;
		std::vector<Path*> pathList;

		for (int k = 0; k < m_lightCount; k++)
		{
			setOnePointEmitter(k);
			findGlintsPathCut(pathList, bounce);
		}

		cout << "start splatting..." << endl;
		clock_t t;
		t = clock();

		m_camera->splat(pathList,m_scene);

		for (int i = 0; i < pathList.size(); i++)
			delete pathList[i];
		t = clock() - t;
		printf("Splatting cost %f seconds.\n", ((float)t) / CLOCKS_PER_SEC);
		m_camera->writeOutput(outImageName);

	}

private:
	Camera *m_camera;
	std::vector<MyShape*> m_shapes;
	Vec3 m_lightPos;
	Vec3 m_lightIntensity;
	int m_lightCount;
	std::vector<Vec3> m_lightPosList;
	std::vector<Vec3> m_lightIntensList;
	string m_outImageName;
	int m_nBounce;
	const Emitter *m_env;
	const Scene *m_scene;
	bool HFMesh;
	float m_errorThresh;

};

MTS_NAMESPACE_END

