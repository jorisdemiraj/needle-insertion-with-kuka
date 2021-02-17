#include "Tissue.h"

using namespace std;

Tissue::Tissue()
{
	_N = 0;
	_center_pos.setZero();
}

Tissue::~Tissue()
{
	// placeholder
}

void Tissue::init(void)
{
	_dummy_rederer_handler = simCreateDummy(0.05f, NULL);
	simSetObjectName(_dummy_rederer_handler, "Tissue_D");
	simSetObjectIntParameter(_dummy_rederer_handler, sim_objectproperty_selectinvisible, 0);
	return;
}


void Tissue::addLayer(std::string name, float t, float k, float b, float p_t_percetage, Eigen::Vector3f color)
{
	Layer temp_l;

	if (p_t_percetage < 0.0f)
		p_t_percetage = 0.0f;
	else if (p_t_percetage > 1.0f)
		p_t_percetage = 1.0f;

	temp_l._thick					= t;
	temp_l._name					= name;
	temp_l._K						= k;
	temp_l._B						= b;
	temp_l._perforation_thickness	= t * p_t_percetage;
	temp_l._color					= color;
	temp_l._is_perforated			= false;
	temp_l._touched					= false;

	_layers.push_back(temp_l);
	_N++;

	// ADD CUBES HERE
}

void Tissue::getLayerParams(std::string name, float& out_t, float& out_k, float& out_b)
{
	//! WILL THIS WORK?
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return;
	}

	out_t = _layers[idx]._thick;
	out_k = _layers[idx]._K;
	out_b = _layers[idx]._B;
}


void Tissue::getLayerParams(std::string name, float& out_t, float& out_k, float& out_b, float& out_p_t)
{
	//! WILL THIS WORK?
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return;
	}

	out_t = _layers[idx]._thick;
	out_k = _layers[idx]._K;
	out_b = _layers[idx]._B;
	out_p_t = _layers[idx]._perforation_thickness;
}

void Tissue::getAllLayerParam(Eigen::VectorXf& out_t_vec, Eigen::VectorXf& out_k_vec, Eigen::VectorXf& out_b_vec, Eigen::VectorXf& out_p_t_vec)
{
	out_t_vec.resize(_N);
	out_k_vec.resize(_N);
	out_b_vec.resize(_N);
	out_p_t_vec.resize(_N);

	for (int i = 0; i < _N; i++)
	{
		out_t_vec(i) = _layers[i]._thick;
		out_k_vec(i) = _layers[i]._K;
		out_b_vec(i) = _layers[i]._B;
		out_p_t_vec(i) = _layers[i]._perforation_thickness;
	}
}




bool Tissue::checkPerforation(std::string name)
{
	//! WILL THIS WORK?
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return false;
	}

	if (_layers[idx]._is_perforated)
		return true;
	else
		return false;
}

bool Tissue::checkTouched(std::string name)
{
	//! WILL THIS WORK?
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return false;
	}

	if (_layers[idx]._touched)
		return true;
	else
		return false;
}


void Tissue::printTissue(void)
{
	for (int i = 0; i < _N; i++)
	{
		cout << "Layer " + _layers[i]._name << endl;
		cout << "Thickness:\t" << _layers[i]._thick << endl;
		cout << "K:\t" << _layers[i]._K << endl;
		cout << "B:\t" << _layers[i]._B << endl;
		cout << "Is perforated:\t" << _layers[i]._is_perforated << endl;
		cout << "Has been touched:\t" << _layers[i]._touched << endl << endl;
	}
}

void Tissue::togglePerforation(std::string name)
{
	//! WILL THIS WORK?
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return;
	}
	_layers[idx]._is_perforated = !_layers[idx]._is_perforated;
}

void Tissue::toggleTouched(std::string name)
{
	//! WILL THIS WORK?
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return;
	}

	_layers[idx]._touched = !_layers[idx]._touched;
}

int Tissue::getLayerHandler(std::string name, bool type)
{
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return 0;
	}
	if (type)
		return _layers[idx]._handler;
	else
		return _layers[idx]._static_handler;
}

void Tissue::renderLayers(void)
{

	float depth = _center_pos(2);
	Vector3f curr_cube_pos;
	float sim_curr_cube_pos[3];
	float sim_total_cube_pos[3];

	for (int i = 0; i < _N; i++)
	{
		if (i == 0)
			depth -= (_layers[i]._thick / 2);
		depth -= (_layers[i - 1]._thick / 2) + (_layers[i]._thick / 2);

		curr_cube_pos << 0, 0, depth;
		eigen2SimVec3f(curr_cube_pos, sim_curr_cube_pos);
		float tmp[3] = { _scale[0], _scale[1], _layers[i]._thick };

		//! RESPONDABLE AND DYNAMIC CUBES
		_cube_handlers.push_back(simCreatePureShape(0, 9, tmp, 1.0, NULL)); //HERE
		simSetObjectName(_cube_handlers[i], _layers[i]._name.c_str());
		simSetShapeColor(_cube_handlers[i], NULL, sim_colorcomponent_ambient_diffuse, _layers[i]._color.data());
		simSetObjectPosition(_cube_handlers[i], -1, sim_curr_cube_pos);

		//! NON-RESPONDABLE AND STATIC CUBES
		float transparency[1] = { 0.34f };
		_static_cube_handlers.push_back(simCreatePureShape(0, 1 + 4 + 16, tmp, 1.0f, NULL));
		simSetObjectName(_static_cube_handlers[i], (_layers[i]._name + "_static").c_str());
		simSetShapeColor(_static_cube_handlers[i], NULL, sim_colorcomponent_ambient_diffuse, _layers[i]._color.data());
		simSetShapeColor(_static_cube_handlers[i], NULL, sim_colorcomponent_transparency, transparency);
		simSetObjectPosition(_static_cube_handlers[i], -1, sim_curr_cube_pos);

		_layers[i]._handler = _cube_handlers[i];
		_layers[i]._static_handler = _static_cube_handlers[i];
	}
	float total_depth_total = 0;
	for (int i = 0; i < _N; i++)
		total_depth_total += _layers[i]._thick;
	_d = total_depth_total;

	//float half_total_depth = (depth + _layers[_N - 1]._thick / 2) / 2;
	eigen2SimVec3f(Vector3f(0.0f, 0.0f, (_center_pos(2) - total_depth_total/2 - _layers[0]._thick/2)), sim_total_cube_pos);
	simSetObjectPosition(_dummy_rederer_handler, -1, sim_total_cube_pos);
	for (int i = 0; i < _N; i++)
	{
		simSetObjectParent(_static_cube_handlers[i], _dummy_rederer_handler, true);
		simSetObjectParent(_cube_handlers[i], _static_cube_handlers[i], true);
	}
	simSetObjectPosition(_dummy_rederer_handler, -1, _center_pos.data());

}

void Tissue::reloadLayer(std::string name)
{
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return;
	}

	float T[12];
	float tmp[3] = { _scale[0], _scale[1], _layers[idx]._thick };
	_layers[idx]._handler = simCreatePureShape(0, 9, tmp, 1.0, NULL);
	simSetShapeColor(_layers[idx]._handler, NULL, sim_colorcomponent_ambient_diffuse, _layers[idx]._color.data());
	simGetObjectMatrix(_layers[idx]._static_handler, -1, T);
	simSetObjectMatrix(_layers[idx]._handler, -1, T);
	simSetObjectName(_cube_handlers[idx], _layers[idx]._name.c_str());
	simSetObjectParent(_layers[idx]._handler, _layers[idx]._static_handler, true);

	_layers[idx]._is_perforated = false;
	_layers[idx]._touched = false;
}


float Tissue::getDOP(Eigen::Vector3f start_p, 
	Eigen::Vector3f final_p, 
	Eigen::Vector3f contact_n)
{
	contact_n.normalize();
	Vector3f needle_vector = final_p - start_p;
	float DOP = -needle_vector.dot(contact_n);
	return DOP;
}

int Tissue::getLayerIDXFromTouch(void)
{
	if (_layers[0]._touched == false)
		return -1;
	if (_layers[_N - 1]._touched == true)
		return _N - 1;

	for (int i = 1; i < _N; i++)
	{
		if (_layers[i]._touched == false)
			return i - 1;
	}
	return -1;
}

int Tissue::getLayerIDXFromDepth(Eigen::Vector3f start_p,
	Eigen::Vector3f final_p,
	Eigen::Vector3f contact_n)
{
	float depth = getDOP(start_p, final_p, contact_n);
	if (depth < 0.0f)
		return -1;
	else if (depth >= 0.0f && depth < _layers[0]._thick)
		return 0;
	else if (depth >= _layers[0]._thick && depth < _layers[0]._thick + _layers[1]._thick)
		return 1;
	else if (depth >= _layers[0]._thick + _layers[1]._thick && depth < _layers[0]._thick + _layers[1]._thick + _layers[2]._thick)
		return 2;
	else if (depth >= _layers[0]._thick + _layers[1]._thick + _layers[2]._thick)
		return 3;

	return -1;
}


void Tissue::restoreLayers(int current_layer_idx)
{
	for (int i = current_layer_idx; i < _N; i++)
	{
		if (_layers[i]._handler == NULL_HANDLER)
			reloadLayer(_layers[i]._name);
	}
}


void Tissue::removeDynamicLayer(std::string name)
{
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return;
	}

	simRemoveObject(_layers[idx]._handler);
	_layers[idx]._handler = NULL_HANDLER;
}

void Tissue::resetRendering(void)
{
	for (int i = 0; i < _N; i++)
	{
		simRemoveObject(_layers[i]._handler);
		_layers[i]._handler = -1;

		reloadLayer(_layers[i]._name);
	}
}


void Tissue::setLayerColor(std::string name, Vector3f& color)
{
	int idx = -1;
	bool flag = false;
	for (int i = 0; i < _N; i++)
	{
		if (name == _layers[i]._name)
		{
			flag = true;
			idx = i;
		}
	}
	if (!flag)
	{
		red();
		cerr << "Error, no such tissue layer!" << endl;
		reset();
		return;
	}

	_layers[idx]._color = color;
}
/*
tissue_params[0] <<
1.0,
331,
3;
tissue_params[1] <<
1.0,
83,
1;
tissue_params[2] <<
1.0,
497,
3;
tissue_params[3] <<
1.0,
2483,
0;
*/
