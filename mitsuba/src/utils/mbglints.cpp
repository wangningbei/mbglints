/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/render/util.h>
#include <mitsuba/core/timer.h>
#include <mitsuba/core/fresolver.h>
#include <mitsuba/core/plugin.h>
#include <boost/algorithm/string.hpp>
#if defined(WIN32)
#include <mitsuba/core/getopt.h>
#endif

#include "mbglints/glintbounce.h"
MTS_NAMESPACE_BEGIN


class MultiBounceGlint : public Utility {
public:
	void help() {
		cout << endl;
		cout << "Synopsis: Muliple Bounce Glint Computation" << endl;
		cout << endl;
		cout << "Usage: mtsutil multibounceglint [options] <Scene XML file or PLY file>" << endl;
		cout << "Options/Arguments:" << endl;
		cout << "   -h             Display this help text" << endl << endl;
		cout << "   -n value       Specify the bounce count" << endl << endl;
		cout << "   -m true/false  Use hierarchy for pruning or not" << endl << endl;
	}

	int run(int argc, char **argv) {
		ref<FileResolver> fileResolver = Thread::getThread()->getFileResolver();
		int optchar;
		char *end_ptr = NULL;
		optind = 1;
		int bounceCount = 1;
		bool hierarchy = false;

		/* Parse command-line arguments */
		while ((optchar = getopt(argc, argv, "m:n:h")) != -1) {
			switch (optchar) {
				case 'h': {
						help();
						return 0;
					}
					break;
				case 'n':
					bounceCount = strtol(optarg, &end_ptr,10);
					if (*end_ptr != '\0')
						SLog(EError, "Could not parse the bounce count!");
					break;
				case 'm':
					if (strcmp(optarg, "true") == 0)
						hierarchy = true;
					else if (strcmp(optarg, "false") == 0)
						hierarchy = false;
					else
						SLog(EError, "Could not parse the hierarchy!");
					break;
			};
		}

		ref<Scene> scene;
		ref<ShapeKDTree> kdtree;

		std::string lowercase = boost::to_lower_copy(std::string(argv[optind]));
		if (boost::ends_with(lowercase, ".xml")) {
			fs::path
				filename = fileResolver->resolve(argv[optind]),
				filePath = fs::absolute(filename).parent_path(),
				baseName = filename.stem();
			ref<FileResolver> frClone = fileResolver->clone();
			frClone->prependPath(filePath);
			Thread::getThread()->setFileResolver(frClone);
			scene = loadScene(argv[optind]);
			kdtree = scene->getKDTree();
		} else {
			Log(EError, "The supplied scene filename must end in XML!");
		}

		/* Show some statistics, and make sure it roughly fits in 80cols */
		Logger *logger = Thread::getThread()->getLogger();
		DefaultFormatter *formatter = ((DefaultFormatter *) logger->getFormatter());
		logger->setLogLevel(EDebug);
		formatter->setHaveDate(false);

		scene->initialize();
		optind++;
		string outputImage = std::string(argv[optind]);

		string hfPath = "";
		cout << argc << std::endl;
		cout << outputImage << std::endl;
		BounceGlintRenderer *glintRender;
		cout << "For heightfield input: mtsutil.exe mbglints xml output.exr heightfild.exr heightfieldScale bounceCount errorThresh(0.001)";
		cout << "For mesh input: mtsutil.exe mbglints xml output.exr bounceCount errorThresh(0.001)";
		if (argc == 7){
			cout << "Using a height field. \n";
			optind++;
			hfPath = std::string(argv[optind]);
			optind++;
			float scale = std::stof(argv[optind]);
			//this is a height field object
			glintRender = new BounceGlintRenderer(scene, hfPath, scale);
		}
		else
		{
			cout << "Using a general object. \n";
			//this is a general object
			glintRender = new BounceGlintRenderer(scene);
		}

		optind++;
		int bounceN = std::stoi(argv[optind]);
		optind++;
		float errorThresh = std::stof(argv[optind]);

		clock_t t;
		t = clock();

		glintRender->renderOneImage(outputImage,bounceN, errorThresh); // "path.ply"

		t = clock() - t;
		printf("It cost %f seconds).\n", ((float)t) / CLOCKS_PER_SEC);
		delete glintRender;
		return 0;
	}

	MTS_DECLARE_UTILITY()
};

MTS_EXPORT_UTILITY(MultiBounceGlint, "Multiple Bounce Glint Computation")
MTS_NAMESPACE_END
