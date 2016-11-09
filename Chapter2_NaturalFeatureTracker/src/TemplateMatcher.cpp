#include "TemplateMatcher.hpp"

#include <json/json.h>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <iostream>

struct Candidate
{
	
};

using CandidatesList = std::vector<Candidate>;

bool TemplateMatcher::FindCandidates(cv::Mat inspectionImage, CandidatesList& candidates)
{
	std::vector<cv::KeyPoint> inspectionKeypoints;
	cv::Mat inspectionFeatures;
	mFeatureExtractor->detectAndCompute(inspectionImage, cv::Mat(), inspectionKeypoints, inspectionFeatures);

	matches_map_t matches;
	mTemplatesDb->query(matches, inspectionFeatures);
	// Convert to candidates

	return candidates.size() > 0;
}

bool TemplateMatcher::PruneCandidatesViaHomography(CandidatesList& candidates)
{
	for (int i = 0; i < candidates.size(); i++)
	{
		mHomographyValidator.estimate(candidates[i].h, candidates[i].inliers, candidates[i].matches)
	}
}

bool TemplateMatcher::PruneCandidatesViaRegistration(CandidatesList& candidates)
{

}

bool TemplateMatcher::ConvertToResult(std::vector<DetectionResult>& result, const CandidatesList& candidates)
{
	assert(candidates.empty() == false);
	result.resize(candidates.size());

	return true;
}

bool TemplateMatcher::query(cv::Mat inspectionImage, std::vector<DetectionResult>& result)
{
	CandidatesList candidates;
	if (!FindCandidates(inspectionImage, candidates))
		return false;

	if (!PruneCandidatesViaHomography(candidates))
		return false;
	
	if (!PruneCandidatesViaRegistration(candidates))
		return false;
	
	ConvertToResult(result, candidates);
	return true;
}

std::string normalizePath(std::string path)
{
//#if _MSC_VER
    //int p;
    //while ((p = path.find('/')) != std::string::npos)
    //{
    //  path.replace(p, 1, 1, '\\');
    //}
    //return path;
//#else
    int p;
    while ((p = path.find('\\')) != std::string::npos)
    {
        path.replace(p, 1, 1, '/');
    }
    return path;
//#endif

}

std::string GetDirectoryName(std::string filename)
{
    auto lastSepIdx = filename.find_last_of("\\/");
    if (std::string::npos == lastSepIdx)
        return std::string();
    else
        return normalizePath(filename.substr(0, lastSepIdx));
}

std::string ConcatenatePath(std::string root, std::string rel)
{
    return normalizePath(root + "/" + rel);
}

bool ParseJsonTrainsetImpl(std::vector<DocumentTemplate>& dataset, Json::Value& document, std::string rootDirectory)
{
    if (!document.isArray()) {
        std::cerr << "Root element of JSON file is not array " << std::endl;
        return false;
    }

    for (int index = 0; index < document.size(); index++)
    {
        auto item = document[index];
        //std::cout << item;

        if (!item)
            continue;

        if (!item.isMember("image")) {
            std::cerr << "Missing required 'image' member " << std::endl;            
            return false;
        }

        if (!item.isMember("mask")) {
            std::cerr << "Missing required 'mask' member " << std::endl;            
            return false;            
        }
        
        Json::Value image = item["image"];
        Json::Value mask = item["mask"];

		DocumentTemplate loadedItem;

        if (image.isString())
            loadedItem.Image = image.asString();
        else
            return false;
        
        if (mask.isString())
            loadedItem.Mask = mask.asString();
		else      
            return false;

        loadedItem.Image = ConcatenatePath(rootDirectory, loadedItem.Image);
        loadedItem.Mask = ConcatenatePath(rootDirectory, loadedItem.Mask);
		loadedItem.Markup.State = item["markup"]["state"].asString();
		loadedItem.Markup.Country = item["markup"]["country"].asString();
		loadedItem.Markup.IdType = item["markup"]["idtype"].asString();
		loadedItem.Markup.IdVersion = item["markup"]["idversion"].asString();
		loadedItem.Markup.Region = item["markup"]["region"].asString();

        dataset.push_back(loadedItem);
    }

    return true;
}

bool ParseJsonTrainsetFile(std::vector<DocumentTemplate>& dataset, const std::string& jsonFile)
{
    dataset.clear();
    std::ifstream in(jsonFile.c_str());

    if (!in) {
        std::cerr << "Error opening file " << jsonFile << std::endl;
        return false;        
    }

    Json::Value document;
    Json::Reader reader;
    if (!reader.parse(in, document)) {        
        std::cerr << "Error reading JSON file " << jsonFile << std::endl;
        return false;
    }

	std::string rootDirectory = GetDirectoryName(jsonFile);
	return ParseJsonTrainsetImpl(dataset, document, rootDirectory);
}

bool ParseJsonTrainsetData(std::vector<DocumentTemplate>& dataset, const std::string& jsonData, std::string rootDirectory)
{
    Json::Value document;
    Json::Reader reader;
    if (!reader.parse(jsonData, document)) {        
        std::cerr << "Error reading JSON data " << std::endl;
        return false;
    }

    return ParseJsonTrainsetImpl(dataset, document, rootDirectory);
}

void PrepareTrainset(std::vector<Pattern>& patterns, cv::Feature2D& alg, const std::vector<DocumentTemplate>& templates)
{
	patterns.resize(templates.size());

	for (size_t i = 0; i < templates.size(); i++)
	{
		cv::Mat image = cv::imread(templates[i].Image);
		cv::Mat mask = cv::imread(templates[i].Mask, cv::IMREAD_GRAYSCALE);
		cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

		assert(image.size() == mask.size());
		
		patterns[i].size = image.size();
		patterns[i].frame = image;
		cv::cvtColor(image, patterns[i].grayImg, cv::COLOR_BGR2GRAY);

		alg.detectAndCompute(patterns[i].grayImg, mask, patterns[i].keypoints, patterns[i].descriptors);

		if (1)
		{
			cv::Mat debugImage = image.clone();
			cv::drawKeypoints(image, patterns[i].keypoints, debugImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			int d = 0;
		}

		if (patterns[i].keypoints.empty())
		{
			std::cout << "Cannot detect features for template " << i << std::endl;
		}
	}
}
