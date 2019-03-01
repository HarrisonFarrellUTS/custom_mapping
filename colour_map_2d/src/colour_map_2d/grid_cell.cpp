#include "colour_map_2d/grid_cell.h"

namespace ColourMap2D{

////CONSTRUCTORS
GridCell::GridCell(unsigned index, CellState cell_state)
    : index_(index), cell_state_(cell_state), state_changed_(true)
{
    for(size_t i = 0; i < ColourLib::COLOURS.size() - 1; i++)
    {
        colour_probs_.push_back(1 / static_cast<double>(ColourLib::COLOURS.size() + 1));
    }
    colour_probs_.push_back(2 / static_cast<double>(ColourLib::COLOURS.size() + 1));
}

////GETTERS
unsigned GridCell::getIndex() const {return index_;}
CellState GridCell::getCellState() const {return cell_state_;}
bool GridCell::stateChanged() const {return state_changed_;}

////SETTERS
void GridCell::setIndex(unsigned index) {index_ = index;}
void GridCell::setCellState(CellState cell_state)
{
    cell_state_ = cell_state;
    state_changed_ = true;
}

////METHODS
void GridCell::processPoint(uint8_t r, uint8_t g, uint8_t b, double hit_prob, double miss_prob, double prob_thresh)
{
    if(cell_state_ == CellState::obstacle)
    {
        ColourLib::Colour colour = ColourLib::Identifier::identifyHSVThresh(r, g, b);
        for(size_t i = 0; i < ColourLib::COLOURS.size(); i++)
        {
            if(colour == ColourLib::COLOURS[i].colour) colour_probs_[i] = colour_probs_[i] * hit_prob;
            else colour_probs_[i] = colour_probs_[i] * miss_prob;
        }
        normalizeProbs();
        probThresh(prob_thresh);
    }
    state_changed_ = true;
}
    
void GridCell::processPoint(ColourPoint point, double hit_prob, double miss_prob, double prob_thresh)
{
    if(cell_state_ == CellState::obstacle)
    {
        ColourLib::Colour colour = ColourLib::Identifier::identifyHSVThresh(point.getR(), point.getG(), point.getB());
        for(size_t i = 0; i < ColourLib::COLOURS.size(); i++)
        {
            if(colour == ColourLib::COLOURS[i].colour) colour_probs_[i] = colour_probs_[i] * hit_prob;
            else colour_probs_[i] = colour_probs_[i] * miss_prob;
        }
        normalizeProbs();
        probThresh(prob_thresh);
    }
    state_changed_ = true;
}

void GridCell::normalizeProbs()
{
    double count = 0;
    for(size_t i = 0; i < colour_probs_.size(); i++)
    {
        count += colour_probs_[i];
    }
    for(size_t i = 0; i < colour_probs_.size(); i++)
    {
        colour_probs_[i] = colour_probs_[i] / count;
    }
}

void GridCell::probThresh(double prob_thresh)
{
    for(size_t i = 0; i < colour_probs_.size(); i++)
    {
        if(colour_probs_[i] < prob_thresh) colour_probs_[i] = prob_thresh;
    }
    normalizeProbs();
}

std::pair<ColourLib::Colour, double> GridCell::getMaxProb(bool state_observed)
{
    std::pair<ColourLib::Colour, double> max_prob;
    max_prob.second = 0.0;
    for(size_t i = 0; i < colour_probs_.size(); i++)
    {
        if(colour_probs_[i] > max_prob.second)
        {
            max_prob.second = colour_probs_[i];
            max_prob.first = ColourLib::COLOURS[i].colour;
        }
    }
    if(state_observed) state_changed_ = false;
    return max_prob;
}

void GridCell::printProbs()
{
    for(size_t i = 0; i < colour_probs_.size(); i++)
    {
        std::cout << ColourLib::getName(ColourLib::COLOURS[i].colour) << ": " << colour_probs_[i] << "   ";
    }
    std::cout << std::endl;
}

}

