#pragma once

#include <QCoreApplication>
#include <QCommandLineParser>
#include <QtCore>

#include <iostream>

#include "mesh.h"
#include "meshoperations.h"

// File to wrap around parsing command line options and executing operations
void doPreprocess(QSettings *settings, Mesh *m, MeshOperations *m_o);

void doOversegmentation(QSettings *settings, Mesh *m, MeshOperations *m_o, std::vector<std::vector<int>> &patches);

void doInitialSegmentation(QSettings *settings, Mesh *m, MeshOperations *m_o);

void doRefinedSegmentation(QSettings *settings, Mesh *m, MeshOperations *m_o);

void doFabrication(QSettings *settings, Mesh *m, MeshOperations *m_o);
