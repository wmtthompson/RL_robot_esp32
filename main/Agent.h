/*
 * Agent.h
 *
 *  Created on: Apr 20, 2022
 *      Author: william
 */



#ifndef AGENT_H_
#define AGENT_H_

#include "common.h"

enum CHOOSE_STATE {RANDOM, BEST};

class Agent {
public:
	Agent(double greedyness);
	void policy_lookup(const Scan_State &s1, Action &a1);
	void policy_update(const Scan_State &s1, const Action &a1);


private:
	void setup_indices();
	Action m_actions[27];
	unsigned int m_action_index[3][3][3] = {0};
	CHOOSE_STATE m_chosen_state = BEST;
	uint32_t m_greedyness = 60;
};

#endif /* AGENT_H_ */
