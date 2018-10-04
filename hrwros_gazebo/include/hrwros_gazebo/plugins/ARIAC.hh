/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _ARIAC_HH_
#define _ARIAC_HH_

#include <ostream>
#include <map>
#include <string>
#include <vector>

#include <gazebo/gazebo.hh>

namespace ariac
{
  using namespace gazebo;

  typedef std::string KitType_t;
  typedef std::string TrayID_t;
  typedef std::string OrderID_t;

  /// \brief The score of a tray.
  class TrayScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj TrayScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const TrayScore &_obj)
    {
      _out << "<tray_score " << _obj.trayID << ">" << std::endl;
      _out << "Completion score: [" << _obj.total() << "]" << std::endl;
      _out << "Complete: [" << (_obj.isComplete ? "true" : "false") << "]" << std::endl;
      _out << "Submitted: [" << (_obj.isSubmitted ? "true" : "false") << "]" << std::endl;
      _out << "Part presence score: [" << _obj.partPresence << "]" << std::endl;
      _out << "All parts bonus: [" << _obj.allPartsBonus << "]" << std::endl;
      _out << "Part pose score: [" << _obj.partPose << "]" << std::endl;
      _out << "</tray_score>" << std::endl;
      return _out;
    }
    public: TrayID_t trayID;
            double partPresence = 0.0;
            double allPartsBonus = 0.0;
            double partPose = 0.0;
            bool isComplete = false;  // all parts on the tray
            bool isSubmitted = false;  // the tray has been submitted for evaluation

            /// \brief Calculate the total score.
            double total() const
            {
              return partPresence + allPartsBonus + partPose;
            }
  };

  /// \brief The score of an order.
  class OrderScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj OrderScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const OrderScore &_obj)
    {
      _out << "<order_score " << _obj.orderID << ">" << std::endl;
      _out << "Total order score: [" << _obj.total() << "]" << std::endl;
      _out << "Time taken: [" << _obj.timeTaken << "]" << std::endl;
      _out << "Complete: [" << (_obj.isComplete() ? "true" : "false") << "]" << std::endl;
      for (const auto & item : _obj.trayScores)
      {
        _out << item.second << std::endl;
      }
      _out << "</order_score>" << std::endl;
      return _out;
    }

    /// \brief Mapping between tray IDs and scores.
    public: std::map<TrayID_t, TrayScore> trayScores;

            /// \brief ID of the order being scored.
            OrderID_t orderID;

            /// \brief Time in seconds spend on the order.
            double timeTaken = 0.0;

            /// \brief Calculate if the order is complete.
            /// \return True if all trays have been submitted.
            ///   Will return false if there are no trays in the order.
            bool isComplete() const
            {
              bool isOrderComplete = !this->trayScores.empty();
              for (const auto & item : this->trayScores)
              {
                isOrderComplete &= item.second.isSubmitted;
                if (!isOrderComplete)
                {
                  break;
                }
              }
              return isOrderComplete;
            };

            /// \brief Calculate the total score.
            double total() const
            {
              double total = 0.0;
              for (const auto & item : this->trayScores)
              {
                total += item.second.total();
              }
              return total;
            };
  };

  /// \brief The score of a competition run.
  class GameScore
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj GameScore object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const GameScore &_obj)
    {
      _out << "<game_score>" << std::endl;
      _out << "Total game score: [" << _obj.total() << "]" << std::endl;
      _out << "Total process time: [" << _obj.totalProcessTime << "]" << std::endl;
      _out << "Part travel time: [" << _obj.partTravelTime << "]" << std::endl;
      for (const auto & item : _obj.orderScores)
      {
        _out << item.second << std::endl;
      }
      _out << "</game_score>" << std::endl;
      return _out;
    }

    public: double totalProcessTime = 0.0;
            double partTravelTime = 0.0;
            double planningTime = 0.0;
            double partTravelDistance = 0.0;
            double manipulatorTravelDistance = 0.0;

            // The score of each of the orders during the game.
            std::map<OrderID_t, OrderScore> orderScores;

            /// \brief Calculate the total score.
            double total() const
            {
              double total = 0;
              /*
              total += totalProcessTime;
              total += partTravelTime;
              total += planningTime;
              total += partTravelDistance;
              total += manipulatorTravelDistance;
              */

              for (const auto & item : this->orderScores)
              {
                total += item.second.total();
              }
              return total;
            };
  };

  /// \brief The parameters used for scoring the competition.
  // TODO: this should have a different data type
  class ScoringParameters
  {
    /// \brief Equality comparison operator.
    /// \param[in] sp1 First parameters to compare.
    /// \param[in] sp2 Second parameters to compare.
    /// \return True if sp1 == sp2.
    public: friend bool operator==(const ScoringParameters &sp1, const ScoringParameters &sp2)
    {
      return (
        sp1.objectPresence == sp2.objectPresence &&
        sp1.objectPosition == sp2.objectPosition &&
        sp1.objectOrientation == sp2.objectOrientation &&
        sp1.allObjectsBonusFactor == sp2.allObjectsBonusFactor &&
        sp1.distanceThresh == sp2.distanceThresh);
    }

    /// \brief Inequality comparison operator.
    /// \param[in] sp1 First parameters to compare.
    /// \param[in] sp2 Second parameters to compare.
    /// \return True if sp1 != sp2.
    public: friend bool operator!=(const ScoringParameters &sp1, const ScoringParameters &sp2)
    {
      return !(sp1 == sp2);
    }

    public: double objectPresence = 1.0;
    public: double objectPosition = 0.0;
    public: double objectOrientation = 1.0;

    // Bonus when all objects in the tray: factor * (number of objects)
    public: double allObjectsBonusFactor = 1.0;

    // Acceptable distance in meters to object's target position.
    // The measured distance is between the center of the model and its target,
    // projected onto the tray.
    public: double distanceThresh = 0.03;

    // Acceptable difference in radians to object's target orientation.
    // The measured difference is from a top-down view of the tray, but only if
    // the quaternions are aligned.
    public: double orientationThresh = 0.1;
  };

  /// \brief Determine the model name without namespace
  std::string TrimNamespace(const std::string &modelName)
  {
    // Trim namespaces
    size_t index = modelName.find_last_of('|');
    return modelName.substr(index + 1);
  }

  /// \brief Determine the type of a gazebo model from its name
  std::string DetermineModelType(const std::string &modelName)
  {
    std::string modelType(TrimNamespace(modelName));

    // Trim trailing underscore and number caused by inserting multiple of the same model
    size_t index = modelType.find_last_not_of("0123456789");
    if (modelType[index] == '_' && index > 1)
    {
      modelType = modelType.substr(0, index);
    }

    // Trim "_clone" suffix if exists
    index = modelType.rfind("_clone");
    if (index != std::string::npos)
    {
      modelType.erase(index);
    }

    return modelType;
  }
  
  /// \brief Determine the ID of a gazebo model from its name
  std::string DetermineModelId(const std::string &modelName)
  {
    std::string modelId(TrimNamespace(modelName));

    // Trim trailing underscore and number caused by inserting multiple of the same model
    size_t index = modelId.find_last_not_of("0123456789");
    if (modelId[index] == '_' && index > 1)
    {
      modelId = modelId.substr(index + 1);
    }

    return modelId;
  }

  /// \brief Class to store information about each object contained in a kit.
  class KitObject
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _obj Kit object to output.
    /// \return The output stream
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const KitObject &_obj)
    {
      _out << "<object>" << std::endl;
      _out << "Type: [" << _obj.type << "]" << std::endl;
      _out << "Faulty: [" << (_obj.isFaulty ? "true" : "false") << "]" << std::endl;
      _out << "Pose: [" << _obj.pose << "]" << std::endl;
      _out << "</object>" << std::endl;
      return _out;
    }

    /// \brief Object type.
    public: std::string type;

    /// \brief Whether or not the object is faulty.
    public: bool isFaulty;

    /// \brief Pose in which the object should be placed.
    public: math::Pose pose;

  };

  /// \brief Class to store information about a kit.
  class Kit
  {
    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _kit kit to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Kit &_kit)
    {
      _out << "<kit type='" << _kit.kitType << "'>";
      for (const auto & obj : _kit.objects)
        _out << std::endl << obj;
      _out << std::endl << "</kit>" << std::endl;

      return _out;
    }

    /// \brief The type of the kit.
    public: KitType_t kitType;

    /// \brief A kit is composed of multiple objects.
    public: std::vector<KitObject> objects;
  };

  /// \brief Class to store information about an order.
  class Order
  {
    /// \brief Less than operator.
    /// \param[in] _order Other order to compare.
    /// \return True if this < _order.
    public: bool operator<(const Order &_order) const
    {
      return this->startTime < _order.startTime;
    }

    /// \brief Stream insertion operator.
    /// \param[in] _out output stream.
    /// \param[in] _order Order to output.
    /// \return The output stream.
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Order &_order)
    {
      _out << "<Order>" << std::endl;
      _out << "Start time: [" << _order.startTime << "]" << std::endl;
      _out << "Kits:" << std::endl;
      for (const auto & item : _order.kits)
      {
        _out << item << std::endl;
      }
      _out << "</order>" << std::endl;

      return _out;
    }

    /// \brief The ID of this order.
    public: OrderID_t orderID;

    /// \brief Simulation time in which the order should be triggered.
    public: double startTime;

    /// \brief After how many unwanted parts to interrupt the previous order (-1 for never).
    public: int interruptOnUnwantedParts;

    /// \brief After how many wanted parts to interrupt the previous order (-1 for never).
    public: int interruptOnWantedParts;

    /// \brief Simulation time in seconds permitted for the order to be
    /// completed before cancelling it. Infinite by default.
    public: double allowedTime;

    /// \brief An order is composed of multiple kits of different types.
    public: std::vector<Kit> kits;

    /// \brief Simulation time in seconds spent on this order.
    public: double timeTaken;
  };
}
#endif
