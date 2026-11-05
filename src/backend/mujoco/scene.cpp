// SPDX-License-Identifier: Apache-2.0
#include "scene.hpp"
namespace boltrt {
void configure_studio_scene(mjSpec *spec) {
  // Neutral studio lighting: broad key, restrained camera fill, matte floor.
  for (int i = 0; i < 3; ++i) {
    spec->visual.headlight.ambient[i] = 0.50f;
    spec->visual.headlight.diffuse[i] = 0.12f;
    spec->visual.headlight.specular[i] = 0.12f;
  }
  for (auto element = mjs_firstElement(spec, mjOBJ_LIGHT); element;
       element = mjs_nextElement(spec, element)) {
    mjs_asLight(element)->active = false;
  }
  auto world = mjs_findBody(spec, "world");
  // Sample a broad studio source with six shadow-casting lights. Classic
  // MuJoCo renders each sample separately; their sum softens the penumbra.
  constexpr double offsets[6][2] = {{0.045, 0},  {0.0225, 0.039},   {-0.0225, 0.039},
                                    {-0.045, 0}, {-0.0225, -0.039}, {0.0225, -0.039}};
  for (const auto &offset : offsets) {
    auto key = mjs_addLight(world, nullptr);
    key->type = mjLIGHT_DIRECTIONAL;
    key->pos[0] = 1.8 + offset[0];
    key->pos[1] = 1.2 + offset[1];
    key->pos[2] = 3;
    key->dir[0] = -key->pos[0] / 3;
    key->dir[1] = -key->pos[1] / 3;
    key->dir[2] = -1;
    key->mode = mjCAMLIGHT_TRACKCOM;
    key->castshadow = true;
    for (int i = 0; i < 3; ++i) {
      key->ambient[i] = 0;
      key->diffuse[i] = 0.55f / 6;
      key->specular[i] = 0.25f / 6;
    }
  }
  auto fill = mjs_addLight(world, nullptr);
  fill->type = mjLIGHT_DIRECTIONAL;
  fill->dir[0] = 0.8;
  fill->dir[1] = 0.2;
  fill->dir[2] = -0.5;
  fill->castshadow = false;
  for (int i = 0; i < 3; ++i) {
    fill->ambient[i] = 0;
    fill->diffuse[i] = 0.18f;
    fill->specular[i] = 0.08f;
  }
  auto geom = mjs_addGeom(world, nullptr);
  geom->type = mjGEOM_PLANE;
  geom->size[0] = 0;
  geom->size[1] = 0;
  geom->size[2] = 0.05;
  // A texture gives an infinite visual grid without adding collision geometry.
  auto grid = mjs_addTexture(spec);
  mjs_setName(grid->element, "deploy_floor_grid");
  grid->type = mjTEXTURE_2D;
  grid->builtin = mjBUILTIN_FLAT;
  grid->mark = mjMARK_EDGE;
  grid->width = grid->height = 128;
  const double floor_rgb[3] = {20.0 / 255, 21.0 / 255, 23.0 / 255};
  const double line_rgb[3] = {40.0 / 255, 42.0 / 255, 45.0 / 255};
  for (int i = 0; i < 3; ++i) {
    grid->rgb1[i] = grid->rgb2[i] = floor_rgb[i];
    grid->markrgb[i] = line_rgb[i];
    geom->rgba[i] = 1;
  }
  geom->rgba[3] = 1;
  auto material = mjs_addMaterial(spec, nullptr);
  mjs_setName(material->element, "deploy_floor_material");
  for (int i = 0; i < mjNTEXROLE; ++i)
    mjs_appendString(material->textures, "");
  mjs_setInStringVec(material->textures, mjTEXROLE_RGB, "deploy_floor_grid");
  material->texuniform = true;
  material->texrepeat[0] = material->texrepeat[1] = 4;
  material->specular = 0;
  material->reflectance = 0.22f;
  mjs_setString(geom->material, "deploy_floor_material");
  spec->visual.quality.shadowsize = 8192;
  spec->visual.map.shadowclip = 6;
  mjsTexture *sky = nullptr;
  for (auto element = mjs_firstElement(spec, mjOBJ_TEXTURE); element;
       element = mjs_nextElement(spec, element)) {
    auto texture = mjs_asTexture(element);
    if (texture->type == mjTEXTURE_SKYBOX) {
      sky = texture;
      break;
    }
  }
  if (!sky)
    sky = mjs_addTexture(spec);
  sky->type = mjTEXTURE_SKYBOX;
  sky->builtin = mjBUILTIN_FLAT;
  sky->width = 64;
  sky->height = 384;
  mjs_setString(sky->file, "");
  const double sky_rgb[3] = {12.0 / 255, 13.0 / 255, 15.0 / 255};
  for (int i = 0; i < 3; ++i) {
    sky->rgb1[i] = sky->rgb2[i] = sky_rgb[i];
    spec->visual.rgba.haze[i] = sky_rgb[i];
  }
  spec->visual.rgba.haze[3] = 0;
}
} // namespace boltrt
