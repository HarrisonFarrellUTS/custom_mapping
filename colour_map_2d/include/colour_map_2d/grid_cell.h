#include "colour_map_2d/colour_point.h"
#include "colour_lib/colour.h"
#include <vector>
#include <utility>

#ifndef GRIDCELL_H
#define GRIDCELL_H

namespace ColourMap2D{

enum class CellState{
    obstacle,
    occupied,
    empty,
    unknown
};

class GridCell
{
private:
    unsigned index_;
    bool occupied_;
    CellState cell_state_;
    std::vector<double> colour_probs_;

    bool state_changed_;
public:
    GridCell(unsigned index, CellState cell_state = CellState::unknown);

    ////GETTERS
    unsigned getIndex() const;
    int getCol() const;
    int getRow() const;
    CellState getCellState() const;
    bool stateChanged() const;

    ////SETTERS
    void setIndex(unsigned index);
    void setCellState(CellState cell_state);

    ////METHODS
    void processPoint(uint8_t r, uint8_t g, uint8_t b, double hit_prob, double miss_prob, double prob_thresh);
    void processPoint(ColourPoint point,  double hit_prob, double miss_prob, double prob_thresh);
    void normalizeProbs();
    void probThresh(double prob_thresh);
    std::pair<ColourLib::Colour, double> getMaxProb(bool state_observed = true);
    double getProb(ColourLib::Colour colour);
    void printProbs();

};

}

#endif // GRIDCELL_H
