/**
* \author Xuan Zhang
* \data Mai-July 2018
*/
#ifndef SEMANTICS_MAX_H
#define SEMANTICS_MAX_H

#include <octomap/ColorOcTree.h>
#include <unordered_map>
#include <chrono>

namespace octomap
{

  struct SemanticsMaxLabel
  {
    int label;
    int confidence;
    long timestamp;
    long start_time;
    bool modified;
    std::unordered_map<int, int> labels;
    ColorOcTreeNode::Color semantic_color; ///<Semantic color

    SemanticsMaxLabel() : label(-1), confidence(1), semantic_color() {
      modified = false;
      labels = std::unordered_map<int, int>();
      timestamp = 0;
      semantic_color = ColorOcTreeNode::Color(255, 255, 255);
    }

    bool operator==(const SemanticsMaxLabel& rhs) const
    {
        return label == rhs.label;
    }

    int getLabel () const
    {
      return label;
    }

    int getConfidence () const
    {
      return confidence;
    }

    long getTimestamp() const {
      return timestamp;
    }

    bool isSemanticsSet() const
    {
      if(label >= 0)
        return true;
      return false;
    }

    ColorOcTreeNode::Color getSemanticColor() const
    {
      return semantic_color;
    }

    static SemanticsMaxLabel semanticFusion(SemanticsMaxLabel s1, SemanticsMaxLabel s2)
    {
      if (s1.labels.find(s2.label) == s1.labels.end())
        s1.labels[s2.label] = 1;
      else
        s1.labels[s2.label]++;
      if (s2.timestamp > s1.timestamp)
        s1.timestamp = s2.timestamp;
      s1.start_time = s2.start_time;
      s1.modified = true;
      return s1;
    }

    static SemanticsMaxLabel selectMaxLabel(SemanticsMaxLabel s, std::unordered_map<int, ColorOcTreeNode::Color>& colorMap) {
      // if (!s.modified)
      //   return s;

      //std::cout << "1 Label: " << s.label << " Confidence: " << s.confidence << std::endl;
      
      int max_freq = 0;
      int max_label = 0;

      for (auto it = s.labels.begin(); it != s.labels.end(); it++) {
        if (it->second > max_freq) {
          max_freq = it->second;
          max_label = it->first;
        }
        it->second = 0;
      }

      //std::cout << "max label: " << max_label << std::endl;

      if (s.label == -1 || s.label == 0) {
        //unlabeled or unknown
        s.label = max_label;
        s.confidence = 1;
      }
      else if (s.label == max_label) {
        //same label
        s.confidence = std::min(2, s.confidence + 1);
        s.label = max_label;
      }
      else {
        //different label
        s.confidence--;
        if (s.confidence == 0) {
          s.label = max_label;
          s.confidence = 1;
          //std::cout << "Changed label" << std::endl;
        }

      }

      if (s.label != 0) {
        if (colorMap.find(s.label) == colorMap.end()) {
          s.semantic_color.r = rand() % 256;
          s.semantic_color.g = rand() % 256;
          s.semantic_color.b = rand() % 256;
          colorMap[s.label] = s.semantic_color;
        }
        else
          s.semantic_color = colorMap[s.label];
      }
      else
        s.semantic_color = ColorOcTreeNode::Color(255, 255, 255);

      //std::cout << "2 Label: " << s.label << " Confidence: " << s.confidence << std::endl;
      s.modified = false;
      return s;
    }

  };


  /// Structure contains semantic colors and their confidences
  struct SemanticsMax
  {
    ColorOcTreeNode::Color semantic_color; ///<Semantic color
    float confidence;

    SemanticsMax():semantic_color(), confidence(0.){}

    bool operator==(const SemanticsMax& rhs) const
    {
        return semantic_color == rhs.semantic_color
                && confidence == rhs.confidence;
    }

    bool operator!=(const SemanticsMax& rhs) const
    {
        return !(*this == rhs);
    }

    ColorOcTreeNode::Color getSemanticColor() const
    {
      return semantic_color;
    }

    bool isSemanticsSet() const
    {
      if(semantic_color != ColorOcTreeNode::Color(255,255,255))
        return true;
      return false;
    }

    /// Perform max fusion
    static SemanticsMax semanticFusion(const SemanticsMax s1, const SemanticsMax s2)
    {
      SemanticsMax ret;
      // If the same color, update the confidence to the average
      if(s1.semantic_color == s2.semantic_color)
      {
        ret.semantic_color = s1.semantic_color;
        ret.confidence = (s1.confidence + s2.confidence) / 2.;
      }
      // If color is different, keep the larger one and drop a little for the disagreement
      else
      {
        ret = s1.confidence > s2.confidence ? s1 : s2;
        ret.confidence *= 0.9;
      }
      return ret;
    }
  };

  std::ostream& operator<<(std::ostream& out, SemanticsMax const& s);
}
#endif //SEMANTICS_MAX_H
