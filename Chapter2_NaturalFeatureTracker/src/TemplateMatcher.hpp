#pragma once

#include "Pattern.hpp"
#include "Correspondences.hpp"
#include "ImageDatabase.hpp"
#include "ImageRegistrationMatchValidator.hpp"
#include "HomographyMatchValidator.hpp"

#include <vector>
#include <opencv2/features2d.hpp>
#include <memory>

struct MarkupData
{
    std::string State;
    std::string Country;
    std::string IdType;
    std::string IdVersion;
    std::string Region;
};

struct DocumentTemplate
{
    std::string Image;
    std::string Mask;
    MarkupData  Markup;
};

struct DetectionResult
{
	Matrix33			  Homography;
	PointsCorrespondences Correspondences;
	Quadrangle            Region;
	MarkupData			  Metadata;
};

///
/// Top-level interface for querying image againts templates database
///
class TemplateMatcher
{
public:
	///
	///
	/// 
	bool query(cv::Mat inspectionImage, std::vector<DetectionResult>& result);

protected:
	struct Candidate
	{

	};

	using CandidatesList = std::vector<Candidate>;

	bool FindCandidates(cv::Mat inspectionImage, CandidatesList& candidates);
	bool PruneCandidatesViaHomography(CandidatesList& candidates);
	bool PruneCandidatesViaRegistration(CandidatesList& candidates);
	bool ConvertToResult(std::vector<DetectionResult>&, const CandidatesList& candidates);

private:
	cv::Ptr<cv::Feature2D>			mFeatureExtractor;
	std::shared_ptr<ImageDatabase>  mTemplatesDb;

	HomographyMatchValidator        mHomographyValidator;
	ImageRegistrationMatchValidator mRegistrationValidator;
};

bool ParseJsonTrainsetFile(std::vector<DocumentTemplate>& templates, const std::string& jsonFile);
bool ParseJsonTrainsetData(std::vector<DocumentTemplate>& templates, const std::string& jsonData, std::string rootDirectory);

void PrepareTrainset(std::vector<Pattern>& patterns, cv::Feature2D& alg, const std::vector<DocumentTemplate>& templates);

bool SaveJsonTrainsetFile(const std::vector<Pattern>& templates, const std::string& jsonFile);
bool LoadJsonTrainsetFile(std::vector<Pattern>& templates, const std::string& jsonFile);
