/* Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
 */

#ifndef OLC_SPRINT_HPP
#define OLC_SPRINT_HPP

#include "ContestDijkstra.hpp"

/**
 * Specialisation of Dijkstra for OLC Sprint (also known as OLC League) rules
 */
class OLCSprint : public ContestDijkstra {
public:
  /**
   * Constructor
   */
  OLCSprint(const Trace &_trace);

private:
  gcc_pure
  unsigned FindStart() const;

protected:
  /* virtual methods from AbstractContest */
  virtual ContestResult CalculateResult() const;

  /* virtual methods from NavDijkstra */
  virtual void AddEdges(ScanTaskPoint origin);

  /* virtual methods from ContestDijkstra */
  virtual void UpdateTrace(bool force);
  virtual void AddStartEdges();
};

#endif
