"""Consolidate F3 comparisons without confusing benchmark coverage with success."""
import argparse
import json
from pathlib import Path
import numpy as np
from .reid import CANDIDATES


def summarize(root, corpus, output):
    corpus_data = json.loads(corpus.read_text())
    cases = [Path(c['run']).name for c in corpus_data['results']]
    methods = []
    for candidate in CANDIDATES:
        reports = []
        missing = []
        for case in cases:
            path = root/candidate/case/'score.json'
            if not path.exists():
                missing.append(case)
            else:
                reports.append({'case':case, **json.loads(path.read_text())})
        def aggregate(key):
            fields = ('frames','leader_eligible_frames','correct','wrong','rejected',
                      'absent_frames','absent_false_accepts')
            totals = {field:sum(r[key][field] for r in reports) for field in fields}
            totals['leader_pick_accuracy'] = totals['correct']/totals['leader_eligible_frames'] if totals['leader_eligible_frames'] else None
            return totals
        methods.append({'candidate':candidate,'measured_cases':len(reports),'missing_cases':missing,
                        'nearby':aggregate('nearby'),'all_visible':aggregate('all_visible'),
                        'median_case_embedding_ms':float(np.median([r['processing_median_ms'] for r in reports
                                                                  if r['processing_median_ms'] is not None])) if reports else None,
                        'cases':reports})
    complete = all(not method['missing_cases'] for method in methods)
    ranking = sorted(methods,key=lambda m:((m['nearby']['leader_pick_accuracy'] or 0),
                                          -(m['nearby']['wrong'])),reverse=True)
    result = {'status':'comparison_complete' if complete else 'incomplete',
              'f3_status':'not_qualified', 'methods':methods,
              'ranking_by_nearby_accuracy':[r['candidate'] for r in ranking],
              'provisional_best':ranking[0]['candidate'] if complete else None,
              'selection_scope':'fixed gallery recognition only; deployment requires integrated lock validation',
              'limitations':[('F3 outfit variants share one body/rig; broader appearance diversity is not qualified'
                              if corpus_data.get('scene_profile') == 'f3' else
                              'original single-character F1/F2 corpus; appearance diversity not qualified'),
                             'one recording per configuration','robot/Thor measurements deferred',
                             'comparison completion does not establish F3 lock acceptance']}
    output.write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('root','corpus','output'):
        parser.add_argument('--'+key,type=Path,required=True)
    a=parser.parse_args();summarize(a.root,a.corpus,a.output)


if __name__ == '__main__':
    main()
