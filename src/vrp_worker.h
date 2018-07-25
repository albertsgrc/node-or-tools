#ifndef NODE_OR_TOOLS_VRP_WORKER_C6DF0F45B324_H
#define NODE_OR_TOOLS_VRP_WORKER_C6DF0F45B324_H

#include <nan.h>

#include "adaptors.h"
#include "types.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <utility>
#include <vector>
#include <iostream>

using namespace std;

struct RoutingSolution {
  std::int64_t cost;
  std::vector<std::vector<NodeIndex>> routes;
  std::vector<std::vector<int32_t>> times;
};


struct VRPWorker final : Nan::AsyncWorker {
  using Base = Nan::AsyncWorker;

  VRPWorker(std::shared_ptr<const CostMatrix> costs_,         //
            std::shared_ptr<const DurationMatrix> durations_, //
            std::shared_ptr<const TimeWindows> timeWindows_,  //
            std::shared_ptr<const DemandMatrix> demands_,     //
            Nan::Callback* callback,                          //
            const RoutingModelParameters& modelParams_,       //
            const RoutingSearchParameters& searchParams_,     //
            std::int32_t numNodes_,                           //
            std::int32_t numVehicles_,                        //
            std::int32_t vehicleDepot_,                       //
            std::int32_t timeHorizon_,                        //
            std::int32_t vehicleCapacity_,                    //
            std::int32_t ignoreCapacityLimit_,
            std::int32_t minimumPenalizeDelayMinutes_,
            std::int32_t _freeDelayPenalization,
            std::int32_t _startDelayPenalization,
            std::int32_t _timePenalization,
            std::int32_t _endDelayPenalization,

            std::int32_t _forceGlobalSchedule,
            std::int32_t _forceTimeWindows,
            std::int32_t _maxDeliveryPointsPerVehicle,
            std::int32_t _computeFromIndex,
            std::int32_t _computeUntilIndex,

            std::int32_t _finalTime,
            std::int32_t _finalTimeDelayPenalization,
            DeliveryPriorities deliveryPriorities_, // Delivery point priorities. Number From 0 to 10

            RouteLocks routeLocks_,                           //
            Pickups pickups_,                                 //
            Deliveries deliveries_)                           //
      : Base(callback),
        // Cached vectors and matrices
        costs{std::move(costs_)},
        durations{std::move(durations_)},
        timeWindows{std::move(timeWindows_)},
        demands{std::move(demands_)},
        // Search settings
        numNodes{numNodes_},
        numVehicles{numVehicles_},
        vehicleDepot{vehicleDepot_},
        timeHorizon{timeHorizon_},
        vehicleCapacity{vehicleCapacity_},
        ignoreCapacityLimit{ignoreCapacityLimit_},
        minimumPenalizeDelayMinutes{minimumPenalizeDelayMinutes_},
        freeDelayPenalization{_freeDelayPenalization},
        startDelayPenalization{_startDelayPenalization},
        timePenalization{_timePenalization},
        endDelayPenalization{_endDelayPenalization},

        forceGlobalSchedule{_forceGlobalSchedule},
        forceTimeWindows{_forceTimeWindows},
        maxDeliveryPointsPerVehicle{_maxDeliveryPointsPerVehicle},
        computeFromIndex{_computeFromIndex},
        computeUntilIndex{_computeUntilIndex},

        finalTime{_finalTime},
        finalTimeDelayPenalization{_finalTimeDelayPenalization},

        routeLocks{std::move(routeLocks_)},
        pickups{std::move(pickups_)},
        deliveries{std::move(deliveries_)},
        deliveryPriorities{std::move(deliveryPriorities_)},
        // Setup model
        model{numNodes, numVehicles, NodeIndex{vehicleDepot}, modelParams_},
        modelParams{modelParams_},
        searchParams{searchParams_} {

    const auto costsOk = costs->dim() == numNodes;
    const auto durationsOk = durations->dim() == numNodes;
    const auto timeWindowsOk = timeWindows->size() == numNodes;
    const auto demandsOk = demands->dim() == numNodes;

    if (!costsOk || !durationsOk || !timeWindowsOk || !demandsOk)
      throw std::runtime_error{"Expected costs, durations, timeWindow and demand sizes to match numNodes"};

    const auto routeLocksOk = (std::int32_t)routeLocks.size() == numVehicles;

    if (!routeLocksOk)
      throw std::runtime_error{"Expected routeLocks size to match numVehicles"};

    for (const auto& locks : routeLocks) {
      for (const auto& node : locks) {
        const auto nodeInBounds = node >= 0 && node < numNodes;

        if (!nodeInBounds)
          throw std::runtime_error{"Expected nodes in route locks to be in [0, numNodes - 1]"};

        const auto nodeIsDepot = node == vehicleDepot;

        if (nodeIsDepot)
          throw std::runtime_error{"Expected depot not to be in route locks"};
      }
    }

    const auto pickupsAndDeliveriesOk = pickups.size() == deliveries.size();

    if (!pickupsAndDeliveriesOk)
      throw std::runtime_error{"Expected pickups and deliveries parallel array sizes to match"};
  }

  string time_to_string(int32_t x, bool showSeconds = true) {
    int hours = x/3600;
    int minutes = (x%3600)/60;
    int secondslol = (x%3600)%60;

    string secondsString = ":";
    secondsString += (secondslol < 10 ? "0" : "");
    secondsString += to_string(secondslol);

    return (hours < 10 ? "0" : "") + to_string(hours) + ':' + (minutes < 10 ? "0" : "") + to_string(minutes) + (showSeconds ? secondsString : "");
  }


  void PrintInput() {
      cerr << "NumNodes: " << numNodes << endl;
      cerr << "NumVehicles: " << numVehicles << endl;
      cerr << "VehicleDepot: " << vehicleDepot << endl;
      cerr << "TimeHorizon: " << time_to_string(timeHorizon, true) << endl;
      cerr << "VehicleCapacity: " << vehicleCapacity << endl;

    cerr << "Costs:" << endl;
    costs->print(cerr);
    cerr << endl;

    cerr << "Durations:" << endl;
    durations->print(cerr);
    cerr << endl;

    cerr << "TimeWindows" << endl;
    for (int i = 0; i < timeWindows->size(); ++i) {
        auto x = timeWindows->at(i);
        cerr << (x.start != -1 ? time_to_string(x.start, false) : "     ") << " -> " << (x.stop != -1 ? time_to_string(x.stop, false) : "     ") << endl;
    }

    cerr << "Demands" << endl;
    for (int i = 0; i < demands->dim(); ++i) {
        cerr << demands->at(i, 0) << endl;
    }

  }

  void Execute() override {
    //this->PrintInput();

    auto costAdaptor = makeBinaryAdaptor(*costs);
    auto costCallback = makeCallback(costAdaptor);

    model.SetArcCostEvaluatorOfAllVehicles(costCallback);

    // Time Dimension

    auto durationAdaptor = makeBinaryAdaptor(*durations);
    auto durationCallback = makeCallback(durationAdaptor);

    const static auto kDimensionTime = "time";

    model.AddDimension(durationCallback, timeHorizon, timeHorizon, /*fix_start_cumul_to_zero=*/forceGlobalSchedule != 0, kDimensionTime);
    const auto& timeDimension = model.GetDimensionOrDie(kDimensionTime);
    auto* mutableTimeDimension = model.GetMutableDimension(kDimensionTime);


    int32_t min = timeHorizon;



    for (std::int32_t node = 0; node < numNodes; ++node) {
        auto v = timeWindows->at(node).start;
        if (v != -1 and v < min) min = v;
    }

      for (int j = 0; j < numVehicles; ++j) {
        model.AddVariableMaximizedByFinalizer(
            timeDimension.CumulVar(model.Start(j)));
        model.AddVariableMinimizedByFinalizer(
            timeDimension.CumulVar(model.End(j)));

        mutableTimeDimension->SetEndCumulVarSoftUpperBound(j, finalTime, finalTimeDelayPenalization);
      }


    for (std::int32_t node = 0; node < numNodes; ++node) {
      const auto interval = timeWindows->at(node);

      int priority = deliveryPriorities[node];

      if (interval.start != -1) {
        timeDimension.CumulVar(node)->SetMin(interval.start);

        mutableTimeDimension->SetCumulVarSoftUpperBound(model.IndexToNode(node), interval.start + 60*minimumPenalizeDelayMinutes, startDelayPenalization*priority*priority);

        model.AddVariableMinimizedByFinalizer(timeDimension.CumulVar(node));
      }
      else {
        model.AddVariableMinimizedByFinalizer(timeDimension.CumulVar(node));

        mutableTimeDimension->SetCumulVarSoftUpperBound(model.IndexToNode(node), min, freeDelayPenalization*priority*priority);

        if (!forceGlobalSchedule) model.SlackVar(node, kDimensionTime)->SetMax(0);
      }

      if (interval.stop != -1) {
         mutableTimeDimension->SetCumulVarSoftUpperBound(model.IndexToNode(node), interval.stop, endDelayPenalization*priority*priority);
      }

      // At the moment we only support a single interval for time windows.
      // We can support multiple intervals if we sort intervals by start then stop.
      // Then Cumulval(n)->SetRange(minStart, maxStop), then walk over intervals
      // removing intervals between active intervals:
      // CumulVar(n)->RemoveInterval(stop, start).
    }


/*
    for (std::int32_t node = 0; node < numNodes; ++node) {
        model.AddVariableMinimizedByFinalizer(timeDimension.CumulVar(node));
    }*/


    model.SetDimensionTransitCost(kDimensionTime, timePenalization);

    // Delay Dimension




    // Capacity Dimension

    if (!ignoreCapacityLimit) {
        auto demandAdaptor = makeBinaryAdaptor(*demands);
        auto demandCallback = makeCallback(demandAdaptor);

        const static auto kDimensionCapacity = "capacity";

        model.AddDimension(demandCallback, /*slack=*/0, vehicleCapacity, /*fix_start_cumul_to_zero=*/true, kDimensionCapacity);
        //const auto& capacityDimension = model.GetDimensionOrDie(kDimensionCapacity);
    }


    auto* solver = model.solver();


    // Pickup and Deliveries
    /*
    for (std::int32_t atIdx = 0; atIdx < pickups.size(); ++atIdx) {
      const auto pickupIndex = model.NodeToIndex(pickups.at(atIdx));
      const auto deliveryIndex = model.NodeToIndex(deliveries.at(atIdx));

      auto* sameRouteCt = solver->MakeEquality(model.VehicleVar(pickupIndex),    //
                                               model.VehicleVar(deliveryIndex)); //

      auto* pickupBeforeDeliveryCt = solver->MakeLessOrEqual(timeDimension.CumulVar(pickupIndex),    //
                                                             timeDimension.CumulVar(deliveryIndex)); //

      solver->AddConstraint(sameRouteCt);
      solver->AddConstraint(pickupBeforeDeliveryCt);

      model.AddPickupAndDelivery(pickups.at(atIdx), deliveries.at(atIdx));
    }*/


    /*
    // Force all vehicles to service

    for (auto vehicle = 0; vehicle < numVehicles; ++vehicle) {
        operations_research::IntVar* const start = model.NextVar(model.Start(vehicle));

        for (auto node = model.Size(); node < model.Size() + model.vehicles(); ++node)
          start->RemoveValue(node);
    }*/




    // Done with modifications to the routing model

    model.CloseModel();

    /*
    // Locking routes into place needs to happen after the model is closed and the underlying vars are established
    const auto validLocks = model.ApplyLocksToAllVehicles(routeLocks, false);

    if (!validLocks)
      return SetErrorMessage("Invalid locks");
    */

    //cerr << timeWindows->at(24).start << ' ' << timeDimension.CumulVar(24) << ' ' << timeWindows->at(24).stop << endl;


    const auto* assignment = model.SolveWithParameters(searchParams);


    if (!assignment || (model.status() != RoutingModel::Status::ROUTING_SUCCESS))
      return SetErrorMessage("Unable to find a solution");

    const auto cost = static_cast<std::int64_t>(assignment->ObjectiveValue());

    std::vector<std::vector<NodeIndex>> routes;
    model.AssignmentToRoutes(*assignment, &routes);

    std::vector<std::vector<std::int32_t>> times;
       int vehicle = 0;
    for (const auto& route : routes) {
      std::vector<std::int32_t> routeTimes;

      if (route.size())
       routeTimes.push_back(static_cast<std::int32_t>(assignment->Value(timeDimension.CumulVar(model.Start(vehicle)))));

      for (const auto& node : route) {
        const auto index = model.NodeToIndex(node);

        const auto* timeVar = timeDimension.CumulVar(index);
        //const auto* delayVar = delayDimension->CumulVar(index);

        //if (node == 24) cerr << timeWindows->at(24).start << ' ' << assignment->Value(timeDimension.CumulVar(index)) << ' ' << timeWindows->at(24).stop << endl;

        const auto value = static_cast<std::int32_t>(assignment->Value(timeVar));
        //const auto value2 = static_cast<std::int32_t>(assignment->Value(delayVar));


        //if (i == route.size() -1) std::cerr << "Time: " << value << " Delay:" << value2  <<  ' ';
        //const auto first = static_cast<std::int32_t>(assignment->Min(timeVar));
        //const auto last = static_cast<std::int32_t>(assignment->Max(timeVar));

        routeTimes.push_back(value);
      }

      if (route.size())
        routeTimes.push_back(static_cast<std::int32_t>(assignment->Value(timeDimension.CumulVar(model.End(vehicle)))));
      ++vehicle;

      //std::cerr << std::endl;


      times.push_back(std::move(routeTimes));
    }

    solution = RoutingSolution{cost, std::move(routes), std::move(times)};
  }

  void HandleOKCallback() override {
    Nan::HandleScope scope;

    auto jsSolution = Nan::New<v8::Object>();

    auto jsCost = Nan::New<v8::Number>(solution.cost);
    auto jsRoutes = Nan::New<v8::Array>(solution.routes.size());
    auto jsTimes = Nan::New<v8::Array>(solution.times.size());

    for (std::size_t i = 0; i < solution.routes.size(); ++i) {
      const auto& route = solution.routes[i];
      const auto& times = solution.times[i];

      auto jsNodes = Nan::New<v8::Array>(route.size());
      auto jsNodeTimes = Nan::New<v8::Array>(times.size());

      for (std::size_t j = 0; j < route.size(); ++j)
        Nan::Set(jsNodes, j, Nan::New<v8::Number>(route[j].value()));

      for (std::size_t j = 0; j < times.size(); ++j) {
        auto jsTime = Nan::New<v8::Number>(times[j]);

        Nan::Set(jsNodeTimes, j, jsTime);
      }

      Nan::Set(jsRoutes, i, jsNodes);
      Nan::Set(jsTimes, i, jsNodeTimes);
    }

    Nan::Set(jsSolution, Nan::New("cost").ToLocalChecked(), jsCost);
    Nan::Set(jsSolution, Nan::New("routes").ToLocalChecked(), jsRoutes);
    Nan::Set(jsSolution, Nan::New("times").ToLocalChecked(), jsTimes);

    const auto argc = 2u;
    v8::Local<v8::Value> argv[argc] = {Nan::Null(), jsSolution};

    callback->Call(argc, argv);
  }

  // Shared ownership: keeps objects alive until the last callback is done.
  std::shared_ptr<const CostMatrix> costs;
  std::shared_ptr<const DurationMatrix> durations;
  std::shared_ptr<const TimeWindows> timeWindows;
  std::shared_ptr<const DemandMatrix> demands;

  std::int32_t numNodes;
  std::int32_t numVehicles;
  std::int32_t vehicleDepot;
  std::int32_t timeHorizon;
  std::int32_t vehicleCapacity;
  std::int32_t ignoreCapacityLimit;
  std::int32_t minimumPenalizeDelayMinutes;
    std::int32_t freeDelayPenalization;
    std::int32_t startDelayPenalization;
    std::int32_t timePenalization;
    std::int32_t endDelayPenalization;

    std::int32_t forceGlobalSchedule;
    std::int32_t forceTimeWindows;
    std::int32_t maxDeliveryPointsPerVehicle;
    std::int32_t computeFromIndex;
    std::int32_t computeUntilIndex;

    std::int32_t finalTime;
    std::int32_t finalTimeDelayPenalization;


  const DeliveryPriorities deliveryPriorities;
  const RouteLocks routeLocks;

  const Pickups pickups;
  const Deliveries deliveries;

  RoutingModel model;
  RoutingModelParameters modelParams;
  RoutingSearchParameters searchParams;

  // Stores solution until we can translate back to v8 objects
  RoutingSolution solution;
};

#endif
