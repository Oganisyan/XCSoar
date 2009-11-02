/* Generated by Together */
#ifndef ORDEREDTASK_H
#define ORDEREDTASK_H

#include "AbstractTask.hpp"
#include "BaseTask/StartPoint.hpp"
#include "BaseTask/FinishPoint.hpp"
#include <vector>
#include "Util/Serialisable.hpp"
#include "GlideSolvers/MacCready.hpp"

class OrderedTaskPoint;

class OrderedTask:
  public AbstractTask,
  public Serialisable
{
public:
  OrderedTask(const TaskEvents &te, 
              const TaskBehaviour &tb,
              const TaskProjection &tp,
              TaskAdvance &ta,
              GlidePolar &gp);
  ~OrderedTask();

  TaskPoint* getActiveTaskPoint();
  virtual void setActiveTaskPoint(unsigned);

  bool insert(OrderedTaskPoint*, unsigned position);
  bool append(OrderedTaskPoint*);
  bool remove(unsigned position);
  bool check_task() const;

  virtual bool update_sample(const AIRCRAFT_STATE &, const bool full_update);

  virtual bool update_idle(const AIRCRAFT_STATE&);

  unsigned task_size() const {
    return tps.size();
  }

  // these used by task dijkstra
  const SearchPointVector& get_tp_search_points(unsigned tp) const {
    return tps[tp]->get_search_points();
  }
  void set_tp_search_min(unsigned tp, const SearchPoint &sol) {
    tps[tp]->set_search_min(sol);
  }
  void set_tp_search_max(unsigned tp, const SearchPoint &sol) {
    tps[tp]->set_search_max(sol);
  }

#ifdef DO_PRINT
  virtual void print(const AIRCRAFT_STATE &state);
#endif

protected:
  virtual bool check_transitions(const AIRCRAFT_STATE &, const AIRCRAFT_STATE&);
  double scan_distance_nominal();
  double scan_distance_planned();
  double scan_distance_remaining(const GEOPOINT &location);
  double scan_distance_scored(const GEOPOINT &location);
  double scan_distance_travelled(const GEOPOINT &location);
  void scan_distance_minmax(const GEOPOINT &location, bool full,
                            double *dmin, double *dmax);

private:

  std::vector<OrderedTaskPoint*> tps;

  void update_geometry();

  /**
   * @supplierCardinality 1 
   */
  StartPoint *ts;

  /**
   * @supplierCardinality 1 
   */
  FinishPoint *tf;

  const TaskProjection &task_projection;

protected:

  virtual double scan_total_start_time(const AIRCRAFT_STATE &);
  virtual double scan_leg_start_time(const AIRCRAFT_STATE &);

  void glide_solution_remaining(const AIRCRAFT_STATE &, 
                                GLIDE_RESULT &total,
                                GLIDE_RESULT &leg);

  void glide_solution_travelled(const AIRCRAFT_STATE &, 
                                GLIDE_RESULT &total,
                                GLIDE_RESULT &leg);

  void glide_solution_planned(const AIRCRAFT_STATE &, 
                              GLIDE_RESULT &total,
                              GLIDE_RESULT &leg,
                              DistanceRemainingStat &total_remaining_effective,
                              DistanceRemainingStat &leg_remaining_effective,
                              const double total_t_elapsed,
                              const double leg_t_elapsed);

  double calc_mc_best(const AIRCRAFT_STATE &);

  double calc_glide_required(const AIRCRAFT_STATE &aircraft);

  double calc_cruise_efficiency(const AIRCRAFT_STATE &aircraft);

  double calc_min_target(const AIRCRAFT_STATE &, 
                         const double t_target);
  virtual double calc_gradient(const AIRCRAFT_STATE &state) ;

private:
  void set_neighbours(unsigned position);
  bool check_startfinish(OrderedTaskPoint* new_tp); 
public:
  void Accept(TaskPointVisitor& visitor) const;
  DEFINE_VISITABLE()
};

#endif //ORDEREDTASK_H
